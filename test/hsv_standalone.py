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
hsv_standalone.py
-----------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Loads images from specified paths.
2. Converts the images from BGR to HSV color space.
3. Saves the H and S channels of the images to specified paths.
4. Converts and prints the HSV values of specific BGR colors.
5. Initializes camera objects with specified parameters.
6. Displays the H and S channels of the left camera image.

Dependencies:
- numpy: Python library for numerical computations.
- pickle: Standard Python library for serializing and de-serializing Python object structures.
- matplotlib: Python library for creating static, animated, and interactive visualizations in Python.
- cv2: OpenCV library for computer vision tasks.
- time: Standard Python library for time-related tasks.
- os: Standard Python library for interacting with the operating system.
- camera_utils: Custom module for camera utilities.
- spatial_utils: Custom module for spatial utilities.
- tracker_utils: Custom module for tracker utilities.

Usage:
Run this script to perform the tasks mentioned above. Ensure that the images are available at the specified paths.
"""

# Define the destination for the images
image_dest = 'D:\\Msc_Project\\BGR_client\\images'

# Load the left image from the specified path
left_image = os.path.join(image_dest, 'left_1.png')
left_cv_img = cv2.imread(left_image)

# Convert the left image from BGR to HSV color space
left_hsv = cv2.cvtColor(left_cv_img, cv2.COLOR_BGR2HSV)

# Load the right image from the specified path
right_image = os.path.join(image_dest, 'right_1.png')
right_cv_img = cv2.imread(right_image)

# Convert the right image from BGR to HSV color space
right_hsv = cv2.cvtColor(right_cv_img, cv2.COLOR_BGR2HSV)

# Save the H and S channels of the left image to the specified paths
cv2.imwrite(os.path.join(image_dest, 'left_h.png'), left_hsv[:, :, 0])
cv2.imwrite(os.path.join(image_dest, 'left_s.png'), left_hsv[:, :, 1])

# Save the H and S channels of the right image to the specified paths
cv2.imwrite(os.path.join(image_dest, 'right_h.png'), right_hsv[:, :, 0])
cv2.imwrite(os.path.join(image_dest, 'right_s.png'), right_hsv[:, :, 1])

# Convert and print the HSV values of specific BGR colors
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

# Initialize camera objects with specified parameters
left_cam = camera_utils.AirsimCamera(640, 360, 80, [2, -0.5, -0.5], [-40.0, -10.0, 0])
right_cam = camera_utils.AirsimCamera(640, 360, 80, [2, 0.5, -0.5], [40.0, -10.0, 0])


# Get the H and S channels of the left camera image
hsv_img = left_hsv
h_channel = hsv_img[:, :, 0]
s_channel = hsv_img[:, :, 1]


# Display the H and S channels of the left camera image
cv2.imshow('H', h_channel)
cv2.imshow('S', s_channel)
cv2.waitKey(1)
a=1
