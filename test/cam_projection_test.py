import numpy as np
import pickle
from matplotlib import pyplot as plt
import camera_utils
import spatial_utils
import tracker_utils
import cv2
import time
import os

"""
cam_projection_test.py
-----------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Loads images from a specified directory.
2. Defines the positions and orientations of the left and right cameras and the LiDAR sensor.
3. Loads the centroids of the detected cones from a pickle file.
4. For each centroid, it crops a region of interest from the image, converts it to the HSV color space, and estimates the color of the cone.
5. Draws a rectangle around the region of interest in the image.
6. Displays the image with the drawn rectangles.

Dependencies:
- numpy: Python library for numerical computations.
- pickle: Standard Python library for serializing and de-serializing Python object structures.
- matplotlib: Python library for creating static, animated, and interactive visualizations in Python.
- camera_utils: Custom module for camera utilities.
- spatial_utils: Custom module for spatial utilities.
- tracker_utils: Custom module for tracking cones.
- cv2: OpenCV library for real-time computer vision.
- time: Standard Python library for time-related tasks.
- os: Standard Python library for interacting with the operating system.

Usage:
Run this script to start the camera projection test. Ensure that the images and the pickle file with the centroids are available in the specified directory.
"""

# Define the directory where the images are stored
image_dest = 'D:\\MscProject\\BGR_client\\images'

# Load the left and right images
left_image = os.path.join(image_dest, 'left_0.png')
left_cv_img = cv2.imread(left_image)
right_image = os.path.join(image_dest, 'right_10.png')
right_cv_img = cv2.imread(right_image)

# Define the HSV values for different colors
blue = np.uint8([[[255, 0, 0]]])
hsv_blue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
print('blue:', hsv_blue)

green = np.uint8([[[0, 255, 0]]])
hsv_green = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
print('green:', hsv_green)

red = np.uint8([[[0, 0, 255]]])
hsv_red = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)
print('red:', hsv_red)

orange = np.uint8([[[0, 165, 255]]])
hsv_orange = cv2.cvtColor(orange, cv2.COLOR_BGR2HSV)
print('orange:', hsv_orange)

yellow = np.uint8([[[0, 255, 255]]])
hsv_yellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2HSV)
print('yellow:', hsv_yellow)

# Define the positions and orientations of the left and right cameras and the LiDAR sensor
left_cam = camera_utils.AirsimCamera(640, 360, 70, [2, -0.5, -0.5], [-40.0, -10.0, 0])
right_cam = camera_utils.AirsimCamera(640, 360, 70, [2, 0.5, -0.5], [40.0, -10.0, 0])
lidar_pos = [2, 0, -0.1]
lidar_rot = [0, 0, 0]

# Calculate the transformation matrices
lidar_to_vehicle = spatial_utils.tf_matrix_from_airsim_pose(lidar_pos, lidar_rot)
left_cam_to_vehicle = left_cam.tf_matrix
right_cam_to_vehicle = right_cam.tf_matrix
lidar_to_left_cam = np.matmul(np.linalg.inv(left_cam_to_vehicle), lidar_to_vehicle)
lidar_to_right_cam = np.matmul(np.linalg.inv(right_cam_to_vehicle), lidar_to_vehicle)

# Load the centroids of the detected cones from a pickle file
with open('camera_centroids.pickle', 'rb') as handle:
    centroid_data = pickle.load(handle)

# Limit the number of centroids to 4 for testing purposes
centroid_data['left'] = centroid_data['left'][:4]
centroid_data['right'] = centroid_data['right'][:4]

# Initialize a dummy tracker
dummy_tracker = tracker_utils.ConeTracker

# Start the timer
tic = time.time()

# For each centroid in the right image, estimate the color of the cone and draw a rectangle around it
for curr_centroid in centroid_data['right']:
    curr_centroid[2] -= 0.05
    hsv_image, success = right_cam.get_cropped_hsv(right_cv_img, curr_centroid)
    cone_color = tracker_utils.estimate_cone_color(hsv_image)
    h_range, w_range = right_cam.generate_cropping_indices(curr_centroid)
    right_cv_img = cv2.rectangle(right_cv_img, [w_range[1], h_range[1]], [w_range[0], h_range[0]], (0, 0, 255), 1)

    print('right:', cone_color)

#     pixel, dist = right_cam.project_vector_to_pixel(curr_centroid)
#     pixel = np.round(pixel).astype(np.uint32)
#     indices = np.flip(pixel)
#     rect_h = np.round(40 / dist).astype(np.uint32)  # Heuristic, half-size of desired rectangle height
#     rect_w = np.round(30 / dist).astype(np.uint32)  # Heuristic, half-size of desired rectangle width
#     cropped_img = right_cv_img[indices[0]-rect_h:indices[0]+rect_h, indices[1]-rect_w:indices[1]+rect_w, :]
#     # cropped_img = right_cv_img[130:430, 200:330, :]
#     hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
#     hsv_size = hsv_img.shape[0] * hsv_img.shape[1]
#     hue_histogram = cv2.calcHist(hsv_img, [0], None, [180], [0, hsv_size])
#     plt.hist(hsv_img[:, :, 0].flatten(), 180)
#     plt.show()
#     rect_extent = np.array([rect_w, rect_h], dtype=np.uint32)
#     right_cv_img = cv2.rectangle(right_cv_img, pixel-rect_extent, pixel+rect_extent, (0, 0, 255), 1)
#     # right_cv_img = cv2.rectangle(right_cv_img, [130, 200], [430, 330], (0, 0, 255), 1)
#     pass

# for curr_centroid in centroid_data['left']:
#     curr_centroid[2] -= 0.05
#     hsv_image = left_cam.get_cropped_hsv(left_cv_img, curr_centroid)
#     cone_color = tracker_utils.estimate_cone_color(hsv_image)
    # h_range, w_range = left_cam.generate_cropping_indices(curr_centroid)
    # left_cv_img = cv2.rectangle(left_cv_img, [w_range[1], h_range[1]], [w_range[0], h_range[0]], (0, 0, 255), 1)
    # print('left:', cone_color)
    # pixel, dist = left_cam.project_vector_to_pixel(curr_centroid)
    # pixel = np.round(pixel).astype(np.uint32)
    # indices = np.flip(pixel)
    # rect_h = np.round(40 / dist).astype(np.uint32)  # Heuristic, half-size of desired rectangle height
    # rect_w = np.round(30 / dist).astype(np.uint32)  # Heuristic, half-size of desired rectangle width
    # cropped_img = left_cv_img[indices[0]-rect_h:indices[0]+rect_h, indices[1]-rect_w:indices[1]+rect_w, :]
    # # cropped_img = right_cv_img[250:260, 170:176, :]
    # hsv_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
    # hsv_size = hsv_img.shape[0] * hsv_img.shape[1]
    # hue_histogram = cv2.calcHist(hsv_img, [0], None, [180], [0, hsv_size])
    # plt.hist(hsv_img[:, :, 0].flatten(), 180)
    # plt.show()
    # rect_extent = np.array([rect_w, rect_h], dtype=np.uint32)
    # left_cv_img = cv2.rectangle(left_cv_img, pixel-rect_extent, pixel+rect_extent, (0, 0, 255), 1)
    # # left_cv_img = cv2.rectangle(left_cv_img, [270, 330], [180, 270], (0, 0, 255), 1)
    pass

# print('time it took:', time.time() - tic)

# Enlarge the right image and display it
enlarged_image = cv2.resize(right_cv_img, (1280, 720))

# enlarged_image = cv2.resize(left_cv_img, (1280, 720))
cv2.imshow('test', enlarged_image)
cv2.waitKey()
a = 5
