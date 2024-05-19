#!/usr/bin/env python
from spline_utils import PathSpline
import numpy as np
import airsim
from scipy.spatial.transform import Rotation as Rot
import time
import pickle
import multiprocessing
from spatial_utils import set_initial_pose
import csv


"""
saving_sensors.py
-----------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Connects to the AirSim simulator.
2. Sets the initial pose of the vehicle.
3. Retrieves Lidar data from the simulator.
4. Reshapes the Lidar data into a point cloud.
5. Saves the point cloud data into a pickle file.
6. Saves the point cloud data into a CSV file.

Dependencies:
- airsim: Python library for the AirSim simulator.
- numpy: Python library for numerical computations.
- pickle: Standard Python library for serializing and de-serializing Python object structures.
- csv: Standard Python library for reading and writing CSV files.
- time: Standard Python library for time-related tasks.
- multiprocessing: Standard Python library for using multiple processors.
- spatial_utils: Custom module for spatial utilities.
- spline_utils: Custom module for spline utilities.

Usage:
Run this script to perform the tasks mentioned above. Ensure that the AirSim simulator is running and the vehicle is set at the correct initial pose.
"""

if __name__ == '__main__':

    # Define the starting point for the AirSim simulator
    # Airsim is stupid, always spawns at zero. Must compensate using "playerstart" in unreal:
    starting_x = 10.0
    starting_y = 20.0

    # connect to the AirSim simulator
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    # Set the initial pose of the vehicle in the simulator
    set_initial_pose(client, [0.0, 0.0], -90.0)
    time.sleep(2.0)

    # Retrieve Lidar data from the simulator
    lidarData = client.getLidarData()
    # Reshape the Lidar data into a point cloud
    point_cloud = np.array(lidarData.point_cloud, dtype=np.dtype('f4'))
    point_cloud = np.reshape(point_cloud, (int(point_cloud.shape[0] / 3), 3))

    # Save the point cloud data into a pickle file
    with open('pointcloud.pickle', 'wb') as pickle_file:
        pickle.dump(point_cloud, pickle_file)
    print('saved pickle data')

    # Save the point cloud data into a CSV file
    with open('pointcloud.csv', 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['x', 'y', 'z'])
        writer.writerows(point_cloud)
    print('saved csv data')
