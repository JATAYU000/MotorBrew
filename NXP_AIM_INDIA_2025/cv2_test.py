import cv2
import numpy as np
import os

print(os.getcwd())
img = cv2.imread('NXPaim/global_slam.png')

np_img = np.load('NXPaim/global.npy')
print(f"Loaded np array shape: {np_img.shape}")
unique_pixels= np.unique(np_img.reshape(-1))
print(f"Unique pixel values: {unique_pixels}")
# count of these unique pixels
pixel_counts = {pixel: np.sum(np_img == pixel) for pixel in unique_pixels}
print(f"Pixel counts: {pixel_counts}")


np_img[np_img == -1] = 255

# 255 is obstacles
# 0 is free space where robot can move
# 95- 100 are obstacles

# find i,j of pixel value 0 in the angle 135 degrees
angle = 135+90
angle_rad = np.deg2rad(angle)
center_i, center_j = np_img.shape[0] // 2, np_img.shape[1] // 2

max_distance = min(np_img.shape)  # maximum possible distance in the image
best_i, best_j = center_i, center_j

for d in range(1, max_distance):
    Rz = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                   [np.sin(angle_rad), np.cos(angle_rad)]])
    # Rotate the vector (d, 0) by the angle
    rotated_vector = Rz @ np.array([d, 0])
    # Calculate the new pixel coordinates
    i = int(center_i + rotated_vector[0])
    j = int(center_j + rotated_vector[1])
    if i < 0 or i >= np_img.shape[0] or j < 0 or j >= np_img.shape[1]:
        break
    pixel = np_img[i, j]
    if pixel in [99, 100, 255]:  # obstacle or unexplored
        # step back to last free space
        i = int(center_i + (d - 3) * np.sin(angle_rad))
        j = int(center_j + (d - 3) * np.cos(angle_rad))
        if np_img[i, j] == 0:
            best_i, best_j = i, j
        break
    if pixel == 0:
        best_i, best_j = i, j

print(f"Best free pixel at angle {angle}°: ({best_i}, {best_j})")

display_img = np_img.copy()
display_img = display_img.astype(np.uint8)
cv2.imshow('Global SLAM Map', display_img)

# 124, 176

print()