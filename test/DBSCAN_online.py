#!/usr/bin/env python
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import time
import pickle
import csv
from matplotlib import pyplot as plt
from sklearn.cluster import DBSCAN
import airsim
import dbscan_utils
from spatial_utils import set_initial_pose

"""
DBSCAN_online.py
-----------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Connects to the AirSim simulator.
2. Sets the initial pose of the vehicle.
3. Runs a loop for a specified duration where it:
    - Aggregates detections from the LiDAR sensor.
    - Filters the point cloud based on specified criteria.
    - Applies the DBSCAN algorithm to the filtered point cloud.
    - Stores the segmentation and centroids of the detected clusters.
4. After the loop, it saves the segmentation and centroids to a pickle file.

Dependencies:
- numpy: Python library for numerical computations.
- scipy.spatial.transform: Module for 3D rotations and transformations.
- time: Standard Python library for time-related tasks.
- pickle: Standard Python library for serializing and de-serializing Python object structures.
- csv: Standard Python library for reading and writing CSV files.
- matplotlib: Python library for creating static, animated, and interactive visualizations in Python.
- sklearn.cluster: Module for clustering data.
- airsim: Python library for the AirSim simulator.
- dbscan_utils: Custom module for DBSCAN utilities.
- spatial_utils: Custom module for spatial utilities.

Usage:
Run this script to start the online DBSCAN test. Ensure that the AirSim simulator is running and the AirSim client is properly set up.
"""

def aggregate_detections(airsim_client, iterations=2):
    # Function to aggregate detections from the LiDAR sensor
    pointcloud = np.array([])
    for curr_iter in range(iterations):
        lidarData = airsim_client.getLidarData()
        pointcloud = np.append(pointcloud, np.array(lidarData.point_cloud, dtype=np.dtype('f4')))
    return np.reshape(pointcloud, (int(pointcloud.shape[0] / 3), 3))


if __name__ == '__main__':

    # Airsim is stupid, always spawns at zero. Must compensate using "playerstart" in unreal:
    starting_x = 10.0
    starting_y = 20.0

    # connect to the AirSim simulator
    client = airsim.CarClient()
    client.confirmConnection()
    # client.enableApiControl(True)
    set_initial_pose(client, [0.0, 0.0], -90.0)
    time.sleep(2.0)

    all_segments = []
    all_centroids = []
    start_time = time.time()
    idx = 0
    while time.time() - start_time < 30:
        # Main loop where the point cloud is aggregated, filtered, and segmented, and the segmentation and centroids are stored
        # Constant throttle 0.1 (speed in the future)
        # trackers over centroids
        # separate into left/right
        # control law for steering
        # add color detection for "shufuni"
        point_cloud = aggregate_detections(client)  # Airsim's stupid lidar implementation
        point_cloud[:, 2] *= -1  # Z is reversed in Airsim because of flying convention
        filtered_pc = dbscan_utils.filter_cloud(point_cloud, 5.0, 20.0, -0.5, 1.0)
        # after it works, dont forget to throw z away

        if filtered_pc.size > 0:
            db = DBSCAN(eps=0.3, min_samples=3).fit(filtered_pc)
            curr_segments, curr_centroids = dbscan_utils.collate_segmentation(db, 0.5)

            if curr_segments:
                all_segments.append(curr_segments)
                all_centroids.append(curr_centroids)

            print(idx)
            idx += 1
            time.sleep(0.1)

    # Save the segmentation and centroids to a pickle file
    pickle_data = {}
    pickle_data['all_segments'] = all_segments
    pickle_data['all_centroids'] = all_centroids
    with open('dbscan_session.pickle', 'wb') as pickle_file:
        pickle.dump(pickle_data, pickle_file)
    print('saved pickle data')
