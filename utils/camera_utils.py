import numpy as np
import spatial_utils
import cv2


"""
camera_utils.py
---------------
This script is part of a simulation project for controlling the camera in AirSim.

It performs the following tasks:

1. Defines a class `AirsimCamera` for handling camera-related operations.
2. The `AirsimCamera` class initializes with camera parameters like width, height, field of view, position, and rotation.
3. The class has methods to generate intrinsic matrix, project a vector to pixel space, generate cropping indices for an image, and get a cropped HSV image.
4. The script also includes utility functions to get a BGR image from an AirSim response and save an image to a file.

Dependencies:
- numpy: Python library for numerical computations.
- cv2: OpenCV library for image processing tasks.
- spatial_utils: Custom module for spatial utilities.

Usage:
Import this module to use the `AirsimCamera` class and its methods for camera-related operations in AirSim. Ensure that the AirSim simulator is running and the camera is set with the correct parameters.
"""

class AirsimCamera:
    # cam_pos and cam_rot should be given in AIRSIM coordinates, as specified in settings.json!
    # Initialize the AirsimCamera class with camera parameters
    def __init__(self, cam_width, cam_height, cam_fov, cam_pos=np.array([0, 0, 0]), cam_rot=np.array([0, 0, 0])):
        self.width = cam_width
        self.height = cam_height
        self.fov = cam_fov
        self.pos = cam_pos
        self.rot = cam_rot
        self.WIDTH_DIST_COEFF = 200
        self.HEIGHT_DIST_COEFF = 100
        # Generate the intrinsic matrix for the camera
        self.intrinsic_matrix = self.generate_intrinsics(cam_width, cam_height, cam_fov)
        # Transformation matrix is calculated in ENG coordinate system!
        self.tf_matrix = spatial_utils.tf_matrix_from_airsim_pose(cam_pos, cam_rot)

    # Generate the intrinsic matrix for the camera
    @staticmethod
    def generate_intrinsics(width, height, horizontal_fov):
        focal_distance = 0.5 * width / (np.tan(np.deg2rad(horizontal_fov) / 2))
        cam_intrinsics = np.zeros(shape=(3, 3))
        cam_intrinsics[0, 0] = focal_distance
        cam_intrinsics[1, 1] = focal_distance
        cam_intrinsics[2, 2] = 1.0
        cam_intrinsics[0, 2] = width / 2
        cam_intrinsics[1, 2] = height / 2
        return cam_intrinsics

    # Project a vector to pixel space
    def project_vector_to_pixel(self, vector):
        cam_coordinates = spatial_utils.eng_to_camera(np.array(vector))
        distance = np.linalg.norm(cam_coordinates)
        normalized_vector = cam_coordinates / cam_coordinates[2]
        pixel_space = np.matmul(self.intrinsic_matrix, normalized_vector)
        # Note that camera pixel space axes are not the same as matrix index representation!
        return pixel_space[:2], distance

    # Generate cropping indices for an image
    def generate_cropping_indices(self, vector):
        pixel, dist = self.project_vector_to_pixel(vector)
        pixel = np.round(pixel).astype(np.int32)
        numpy_indices = np.flip(pixel)  # Changing image and matrix dimension order.
        # Heuristic, half-size of desired rectangle bounds:
        rect_h = np.round(self.HEIGHT_DIST_COEFF / dist).astype(np.int32)
        rect_w = np.round(self.WIDTH_DIST_COEFF / dist).astype(np.int32)
        h_range = [numpy_indices[0] - rect_h, numpy_indices[0] + rect_h]
        w_range = [numpy_indices[1] - rect_w, numpy_indices[1] + rect_w]
        return h_range, w_range, numpy_indices

    # Get a cropped HSV image
    def get_cropped_hsv(self, image, vector):
        h_range, w_range, midpoint = self.generate_cropping_indices(vector)
        h_range = np.clip(h_range, 0, image.shape[0])
        w_range = np.clip(w_range, 0, image.shape[1])
        # Condition below reduces the AOI of the camera to 10-90% of the full image.
        # This makes it less prone (but not immune) to cones being out-of-frame.
        if ((self.height * 0.1) < midpoint[0] < (self.height * 0.9)) and ((self.width * 0.1) < midpoint[1] < (self.width * 0.9)):
            return cv2.cvtColor(image[h_range[0]:h_range[1], w_range[0]:w_range[1], :], cv2.COLOR_BGR2HSV), True
        else:
            return np.empty(shape=(1, 1)), False


# Get a BGR image from an AirSim response
def get_bgr_image(airsim_response):
    # Acquire and reshape image:
    image = np.frombuffer(airsim_response.image_data_uint8, dtype=np.uint8).reshape(airsim_response.height,
                                                                                    airsim_response.width,
                                                                                    3)
    return image


# Save an image to a file
def save_img(airsim_response, file_path):
    # Acquire and reshape image:
    image = get_bgr_image(airsim_response)

    # Write to file. If destination folder does not exist, this line will do nothing:
    cv2.imwrite(file_path, image)

