#!/usr/bin/env python3
"""Standalone tester for the YOLOv11 TFLite model.
This script runs detection on images and saves results to CSV.
"""
import os
import glob
import time
import csv

import cv2
import numpy as np
import yaml
import torch
import torchvision
import tflite_runtime.interpreter as tflite


def xywh2xyxy(x):
    """Converts bounding box from xywh to xyxy format."""
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def non_max_suppression_yolov5(
    prediction,
    conf_thres=0.14,
    iou_thres=0.9,
    classes=None,
    agnostic=False,
    multi_label=False,
    labels=(),
    max_det=300,
    nm=0,
):
    """NMS for YOLOv5 format: [batch, num_anchors, 85] where 85 = x,y,w,h,conf + 80 classes"""
    if isinstance(prediction, (list, tuple)):
        prediction = prediction[0]

    device = prediction.device
    mps = "mps" in device.type
    if mps:
        prediction = prediction.cpu()
    bs = prediction.shape[0]
    nc = prediction.shape[2] - nm - 5
    xc = prediction[..., 4] > conf_thres

    max_wh = 7680
    max_nms = 30000
    time_limit = 0.5 + 0.05 * bs
    redundant = True
    multi_label &= nc > 1
    merge = False

    t = time.time()
    mi = 5 + nc
    output = [torch.zeros((0, 6 + nm), device=prediction.device)] * bs
   
    for xi, x in enumerate(prediction):
        x = x[xc[xi]]

        if labels and len(labels[xi]):
            lb = labels[xi]
            v = torch.zeros((len(lb), nc + nm + 5), device=x.device)
            v[:, :4] = lb[:, 1:5]
            v[:, 4] = 1.0
            v[range(len(lb)), lb[:, 0].long() + 5] = 1.0
            x = torch.cat((x, v), 0)

        if not x.shape[0]:
            continue

        x[:, 5:] *= x[:, 4:5]

        box = xywh2xyxy(x[:, :4])
        mask = x[:, mi:]

        if multi_label:
            i, j = (x[:, 5:mi] > conf_thres).nonzero(as_tuple=False).T
            x = torch.cat((box[i], x[i, 5 + j, None], j[:, None].float(), mask[i]), 1)
        else:
            conf, j = x[:, 5:mi].max(1, keepdim=True)
            x = torch.cat((box, conf, j.float(), mask), 1)[conf.view(-1) > conf_thres]

        if classes is not None:
            x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

        n = x.shape[0]
        if not n:
            continue
        x = x[x[:, 4].argsort(descending=True)[:max_nms]]

        c = x[:, 5:6] * (0 if agnostic else max_wh)
        boxes, scores = x[:, :4] + c, x[:, 4]
        i = torchvision.ops.nms(boxes, scores, iou_thres)
        i = i[:max_det]

        output[xi] = x[i]
        if mps:
            output[xi] = output[xi].to(device)
        if (time.time() - t) > time_limit:
            break

    return output


def non_max_suppression_yolov11(prediction, conf_thres=0.25, iou_thres=0.9, max_det=300):
    """
    NMS for YOLOv11 format
    Input can be:
      - [batch, 84, num_boxes] where 84 = 4 coords + 80 class scores (transposed)
      - [batch, num_boxes, 84] (standard)
      - [batch, num_boxes, 6] where 6 = x1,y1,x2,y2,conf,cls (post-processed)
    """
    if isinstance(prediction, (list, tuple)):
        prediction = prediction[0]
   
    batch_size = prediction.shape[0]
   
    # Handle transposed format [batch, 84, num_boxes] -> [batch, num_boxes, 84]
    if len(prediction.shape) == 3 and prediction.shape[1] in [6, 84] and prediction.shape[2] > prediction.shape[1]:
        prediction = prediction.permute(0, 2, 1)
   
    output = []
   
    for batch_idx in range(batch_size):
        pred = prediction[batch_idx]  # [num_boxes, 84] or [num_boxes, 6]
       
        # If format is [num_boxes, 84] with class scores
        if pred.shape[1] == 84:
            # Split into boxes and class scores
            boxes_xywh = pred[:, :4]  # x_center, y_center, width, height
            class_scores = pred[:, 4:]  # 80 class scores
           
            # Get max class score and index for each box
            class_conf, class_idx = class_scores.max(dim=1)
           
            # Convert xywh to xyxy
            boxes = torch.zeros_like(boxes_xywh)
            boxes[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2  # x1
            boxes[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2  # y1
            boxes[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2  # x2
            boxes[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2  # y2
           
            # Filter by confidence
            mask = class_conf > conf_thres
            boxes = boxes[mask]
            scores = class_conf[mask]
            classes = class_idx[mask]
           
        # If format is [num_boxes, 6] with x1,y1,x2,y2,conf,cls
        elif pred.shape[1] == 6:
            boxes = pred[:, :4]
            scores = pred[:, 4]
            classes = pred[:, 5]
           
            mask = scores > conf_thres
            boxes = boxes[mask]
            scores = scores[mask]
            classes = classes[mask]
        else:
            output.append(torch.zeros((0, 6), device=pred.device))
            continue
       
        if len(boxes) == 0:
            output.append(torch.zeros((0, 6), device=pred.device))
            continue
       
        # Apply NMS
        keep_indices = torchvision.ops.nms(boxes, scores, iou_thres)
        keep_indices = keep_indices[:max_det]
       
        # Create output in [x1, y1, x2, y2, conf, cls] format
        result = torch.zeros((len(keep_indices), 6), device=pred.device)
        result[:, :4] = boxes[keep_indices]
        result[:, 4] = scores[keep_indices]
        result[:, 5] = classes[keep_indices]
       
        output.append(result)
   
    return output


def load_labels(path):
    with open(path, 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        if isinstance(data, dict) and 'names' in data:
            return data['names']
    with open(path, 'r') as f:
        return [ln.strip() for ln in f.readlines() if ln.strip()]


def main():
    # Setup paths
    repo_root = os.path.abspath(os.path.dirname(__file__))
    model_path = os.path.join(repo_root, 'resource/yolo11n_int8.tflite')
    labels_path = os.path.join(repo_root, 'resource/coco.yaml')
    images_dir = os.path.join(repo_root, 'images')
    out_dir = os.path.join(repo_root, 'out')
    os.makedirs(out_dir, exist_ok=True)

    print(f"Loading model and labels...")
    interpreter = tflite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    label_names = load_labels(labels_path)

    # Get model input parameters
    input_h = input_details[0]['shape'][1]
    input_w = input_details[0]['shape'][2]
    int8_input = input_details[0]['dtype'] == np.uint8

    # Process images
    image_paths = sorted(glob.glob(os.path.join(images_dir, '*')))
    print(f"Found {len(image_paths)} images")

    csv_path = os.path.join(out_dir, 'detections.csv')
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['image', 'class_name', 'confidence'])

        for img_path in image_paths:
            print(f"Processing: {img_path}")
            img = cv2.imread(img_path)
            if img is None:
                print('Failed to load image:', img_path)
                continue

            orig_h, orig_w = img.shape[:2]
            # copy for annotated output
            image_for_draw = img.copy()
            resized = cv2.resize(img, (input_w, input_h))
            img_rgb = cv2.cvtColor(resized.astype(np.float32), cv2.COLOR_BGR2RGB)
            img_rgb = img_rgb / 255.0
            inp = np.expand_dims(img_rgb, axis=0).astype(np.float32)

            # Handle quantization if needed
            if int8_input:
                scale, zero_point = input_details[0]['quantization']
                if scale == 0:
                    inp_q = inp.astype(np.uint8)
                else:
                    inp_q = (inp / scale + zero_point).astype(np.uint8)
                interpreter.set_tensor(input_details[0]['index'], inp_q)
            else:
                interpreter.set_tensor(input_details[0]['index'], inp)

            start = time.time()
            interpreter.invoke()
            infer_time = (time.time() - start) * 1000.0

            # Get outputs and process
            outputs = []
            for out in output_details:
                x = interpreter.get_tensor(out['index'])
                if out.get('quantization') and out['quantization'][0] != 0:
                    scale, zero_point = out['quantization']
                    x = (x.astype(np.float32) - zero_point) * scale
                outputs.append(x)

            # Process detections
            for pred in outputs:
                if pred.shape[1] == 84:  # YOLOv11 format
                    pred = pred.transpose(0, 2, 1)  # Transpose to [1, 8400, 84]
                pred_t = torch.tensor(pred)
                
                # Run NMS
                dets = non_max_suppression_yolov11(pred_t, conf_thres=0.25, iou_thres=0.45)

                # Process detections, draw boxes and save CSV
                for det in dets:
                    if len(det):
                        print(f"Found {len(det)} detections")
                        for *xyxy, conf, cls in det:
                            # xyxy are in model input scale (input_w, input_h)
                            x1, y1, x2, y2 = map(float, xyxy)
                            class_id = int(cls)
                            if class_id < len(label_names):
                                class_name = label_names[class_id]
                            else:
                                class_name = str(class_id)
                            # skip low confidence detections
                            if float(conf) < 0.25:
                                continue

                            # scale to original image size
                            # handle both normalized (0..1) outputs and input-pixel outputs
                            max_coord = max(x1, y1, x2, y2)
                            if max_coord <= 1.01:
                                # coordinates are normalized (0..1)
                                ox1 = int(x1 * orig_w)
                                oy1 = int(y1 * orig_h)
                                ox2 = int(x2 * orig_w)
                                oy2 = int(y2 * orig_h)
                            else:
                                # coordinates are in input-pixel units
                                scale_x = orig_w / input_w
                                scale_y = orig_h / input_h
                                ox1 = int(x1 * scale_x)
                                oy1 = int(y1 * scale_y)
                                ox2 = int(x2 * scale_x)
                                oy2 = int(y2 * scale_y)

                            # clip to image bounds and ensure valid integer coords
                            ox1 = max(0, min(int(ox1), orig_w - 1))
                            oy1 = max(0, min(int(oy1), orig_h - 1))
                            ox2 = max(0, min(int(ox2), orig_w - 1))
                            oy2 = max(0, min(int(oy2), orig_h - 1))

                            # ensure minimum box size so it's visible
                            if ox2 <= ox1:
                                ox2 = min(ox1 + 2, orig_w - 1)
                            if oy2 <= oy1:
                                oy2 = min(oy1 + 2, orig_h - 1)

                            # draw rectangle and label with filled background
                            cv2.rectangle(image_for_draw, (ox1, oy1), (ox2, oy2), (0, 255, 0), 2)
                            label = f"{class_name} {float(conf):.2f}"
                            font = cv2.FONT_HERSHEY_SIMPLEX
                            font_scale = 0.5
                            font_thickness = 1
                            (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)
                            # label background coordinates (ensure within image)
                            lx1 = ox1
                            ly1 = max(oy1 - text_h - baseline - 4, 0)
                            lx2 = min(ox1 + text_w + 4, orig_w - 1)
                            ly2 = oy1
                            cv2.rectangle(image_for_draw, (lx1, ly1), (lx2, ly2), (0, 255, 0), -1)
                            cv2.putText(image_for_draw, label, (lx1 + 2, ly2 - baseline - 2), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)

                            # write to CSV
                            writer.writerow([
                                os.path.basename(img_path),
                                class_name,
                                float(conf)
                            ])

            # save annotated image
            out_img_path = os.path.join(out_dir, os.path.basename(img_path))
            cv2.imwrite(out_img_path, image_for_draw)
            print(f'Processed {img_path} -> {out_img_path}')

    print('Done. Detections saved to', csv_path)


if __name__ == '__main__':
    main()
