# Copyright 2025 NXP
# Copyright 2016 Open Source Robotics Foundation, Inc.

import rclpy
from rclpy.node import Node
from synapse_msgs.msg import WarehouseShelf
from synapse_msgs.msg import DetectNotifier
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import pkg_resources
import torch
import torchvision
import time
import yaml
import tflite_runtime.interpreter as tflite

QOS_PROFILE_DEFAULT = 10
PACKAGE_NAME = 'b3rb_ros_aim_india'
GREEN_COLOR = (0, 255, 0)


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
    conf_thres=0.05,  # lowered confidence
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


def non_max_suppression_yolov11(prediction, conf_thres=0.05, iou_thres=0.9, max_det=300):
    """NMS for YOLOv11 format"""
    if isinstance(prediction, (list, tuple)):
        prediction = prediction[0]

    batch_size = prediction.shape[0]

    if len(prediction.shape) == 3 and prediction.shape[1] in [6, 84] and prediction.shape[2] > prediction.shape[1]:
        prediction = prediction.permute(0, 2, 1)

    output = []

    for batch_idx in range(batch_size):
        pred = prediction[batch_idx]

        if pred.shape[1] == 84:
            boxes_xywh = pred[:, :4]
            class_scores = pred[:, 4:]
            class_conf, class_idx = class_scores.max(dim=1)
            boxes = torch.zeros_like(boxes_xywh)
            boxes[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2
            boxes[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2
            boxes[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2
            boxes[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2

            mask = class_conf > conf_thres
            boxes = boxes[mask]
            scores = class_conf[mask]
            classes = class_idx[mask]

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

        keep_indices = torchvision.ops.nms(boxes, scores, iou_thres)
        keep_indices = keep_indices[:max_det]

        result = torch.zeros((len(keep_indices), 6), device=pred.device)
        result[:, :4] = boxes[keep_indices]
        result[:, 4] = scores[keep_indices]
        result[:, 5] = classes[keep_indices]

        output.append(result)

    return output


class ObjectRecognizer(Node):
    def __init__(self):
        super().__init__('object_recognizer')

        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            QOS_PROFILE_DEFAULT)

        self.publisher_shelf_objects = self.create_publisher(
            WarehouseShelf, '/shelf_objects', QOS_PROFILE_DEFAULT)

        self.publisher_object_recog = self.create_publisher(
            CompressedImage, "/debug_images/object_recog", QOS_PROFILE_DEFAULT)
        
        self.detect_mode_subscription = self.create_subscription(
            DetectNotifier,
            '/detect_notifier',
            self.detect_mode_callback,
            QOS_PROFILE_DEFAULT)
        ext_delegate_opts = {}
        ext_delegate_opts = [tflite.load_delegate('/usr/lib/libvx_delegate.so', ext_delegate_opts)]
        
        resource_name_coco = "../../../../share/ament_index/resource_index/coco.yaml"
        resource_path_coco = pkg_resources.resource_filename(PACKAGE_NAME, resource_name_coco)
        resource_name_yolo = "../../../../share/ament_index/resource_index/yolo11m_integer_quant.tflite"
        resource_path_yolo = pkg_resources.resource_filename(PACKAGE_NAME, resource_name_yolo)

        with open(resource_path_coco) as f:
            self.label_names = yaml.load(f, Loader=yaml.FullLoader)['names']

        self.interpreter = tflite.Interpreter(model_path=resource_path_yolo)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        output_shape = self.output_details[0]['shape']
        self.get_logger().info(f"Model output shape: {output_shape}")

        if len(output_shape) == 3:
            if output_shape[1] == 84 and output_shape[2] > 84:
                self.model_type = 'yolov11'
            elif output_shape[1] == 6 and output_shape[2] > 6:
                self.model_type = 'yolov11'
            elif output_shape[2] == 6:
                self.model_type = 'yolov11'
            elif output_shape[2] == 85 or (output_shape[2] > 10 and output_shape[1] > output_shape[2]):
                self.model_type = 'yolov5'
            else:
                self.model_type = 'yolov11'
        else:
            self.model_type = 'yolov11'

    def publish_debug_image(self, publisher, image):
        if image.size:
            message = CompressedImage()
            _, encoded_data = cv2.imencode('.jpg', image)
            message.format = "jpeg"
            message.data = encoded_data.tobytes()
            publisher.publish(message)

    def detect_mode_callback(self, message):
        self.get_logger().info(f"Detect mode set to: {message.detect_mode}")
        self.detect_mode = message.detect_mode

    def camera_image_callback(self, message):
        if not getattr(self, "detect_mode", False):
            return

        np_arr = np.frombuffer(message.data, np.uint8)
        image_orig = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        orig_height, orig_width, _ = image_orig.shape

        input_size = self.input_details[0]['shape'][1]
        image = cv2.resize(image_orig, (input_size, input_size))
        image = image.astype(np.float32)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image /= 255
        img = np.expand_dims(image, axis=0)

        input_detail = self.input_details[0]
        int8 = input_detail["dtype"] == np.uint8
        if int8:
            scale, zero_point = input_detail["quantization"]
            img = (img / scale + zero_point).astype(np.uint8)
       
        self.interpreter.set_tensor(input_detail["index"], img)

        self.interpreter.invoke()

        y = []
        for output in self.output_details:
            x = self.interpreter.get_tensor(output["index"])
            if int8:
                scale, zero_point = output["quantization"]
                x = (x.astype(np.float32) - zero_point) * scale
            y.append(x)

        image *= 255
        image = cv2.cvtColor(image.astype(np.float32), cv2.COLOR_RGB2BGR)

        shelf_objects_message = WarehouseShelf()
        object_count_dict = {}

        # Grid setup
        num_cols, num_rows = 3, 2
        cell_width = orig_width / num_cols
        cell_height = orig_height / num_rows

        def is_in_grid(x_center, y_center):
            """Return True if within 3x2 grid."""
            return 0 <= x_center < orig_width and 0 <= y_center < orig_height

        for pred in y:
            pred = torch.tensor(pred)

            if self.model_type == 'yolov5':
                pred[0][..., :4] *= torch.tensor([input_size, input_size, input_size, input_size], device=pred.device)
                pred = non_max_suppression_yolov5(pred, conf_thres=0.05, iou_thres=0.3, max_det=1000)
            else:
                pred = non_max_suppression_yolov11(pred, conf_thres=0.05, iou_thres=0.1, max_det=1000)

            for i, det in enumerate(pred):
                if len(det):
                    for *xyxy, conf, cls in reversed(det):
                        x1 = int(xyxy[0] * orig_width / input_size)
                        y1 = int(xyxy[1] * orig_height / input_size)
                        x2 = int(xyxy[2] * orig_width / input_size)
                        y2 = int(xyxy[3] * orig_height / input_size)

                        x_center = (x1 + x2) / 2
                        y_center = (y1 + y2) / 2

                        if not is_in_grid(x_center, y_center):
                            continue

                        cls_idx = int(cls)
                        if cls_idx < len(self.label_names):
                            object_name = self.label_names[cls_idx]
                            object_count_dict[object_name] = object_count_dict.get(object_name, 0) + 1

                            cv2.rectangle(image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), GREEN_COLOR, 2)
                            cv2.putText(image, f"{object_name} {float(conf):.2f}",
                                      (int(xyxy[0]), int(xyxy[1]) - 5),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN_COLOR, 2, cv2.LINE_AA)

            image = cv2.resize(image, (orig_width, orig_height))
            self.publish_debug_image(self.publisher_object_recog, image)

        for key, value in object_count_dict.items():
            shelf_objects_message.object_name.append(key)
            shelf_objects_message.object_count.append(value)

        self.publisher_shelf_objects.publish(shelf_objects_message)


def main(args=None):
    rclpy.init(args=args)
    object_recognizer = ObjectRecognizer()
    rclpy.spin(object_recognizer)
    object_recognizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
