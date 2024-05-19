#!/usr/bin/env python
from spline_utils import PathSpline
import numpy as np
import airsim
from scipy.spatial.transform import Rotation as Rot
import time
from matplotlib import pyplot as plt
import csv
import pickle

"""
steer_bump_test.py
------------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Connects to the AirSim simulator.
2. Sets the initial pose of the vehicle.
3. Controls the vehicle to steer with a certain setpoint.
4. Collects the vehicle's steering data over time while it's moving.
5. Plots the steering setpoint and the actual steering over time.

Dependencies:
- airsim: Python library for the AirSim simulator.
- numpy: Python library for numerical computations.
- time: Standard Python library for time-related tasks.
- matplotlib: Python library for creating static, animated, and interactive visualizations in Python.
- spline_utils: Custom module for spline utilities.

Usage:
Run this script to perform the tasks mentioned above. Ensure that the AirSim simulator is running and the vehicle is set at the correct initial pose.
"""
# Function to set the initial pose of the vehicle in the simulator
def set_initial_pose(airsim_client, desired_position, desired_heading):
    # Get the current vehicle pose
    initial_pose = airsim_client.simGetVehiclePose()
    # Create a rotation object for the desired heading
    rot = Rot.from_euler('xyz', [0, 0, desired_heading], degrees=True)
    # Convert the rotation to a quaternion
    quat = rot.as_quat()
    # Set the orientation of the vehicle
    initial_pose.orientation.x_val = quat[0]
    initial_pose.orientation.y_val = quat[1]
    initial_pose.orientation.z_val = quat[2]
    initial_pose.orientation.w_val = quat[3]
    # Set the position of the vehicle
    initial_pose.position.x_val = desired_position[0]
    initial_pose.position.y_val = desired_position[1]
    # initial_pose.position.z_val = desired_position[2]
    # Set the vehicle pose in the simulator
    airsim_client.simSetVehiclePose(initial_pose, ignore_collison=True)


if __name__ == '__main__':

    # Airsim is stupid, always spawns at zero. Must compensate using "playerstart" in unreal:
    # Define the starting point for the AirSim simulator
    starting_x = 15.0
    starting_y = 100.0

    # connect to the AirSim simulator
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    # Set the initial pose of the vehicle
    set_initial_pose(client, [0.0, 0.0], -90.0)
    time.sleep(1.0)
    # client.enableApiControl(False)

    # Initialize the car controls
    car_controls = airsim.CarControls()
    # Initialize the data array
    car_data = np.ndarray(shape=(0, 3))

    # Set the initial steering setpoint
    steer_setpoint = 1.0

    # Start the timer
    start_time = time.time()
    # Collect data for 10 seconds
    while time.time() - start_time < 10.0:
        # Calculate the time difference
        time_diff = time.time() - start_time
        # Change the steering setpoint at certain times
        if time_diff > 3.0:
            steer_setpoint = -1.0
        if time_diff > 6.0:
            steer_setpoint = 1.0
        if time_diff > 9.0:
            steer_setpoint = -1.0

        # Set the steering setpoint
        car_controls.steering = steer_setpoint
        # Apply the car controls
        client.setCarControls(car_controls)

        # Get the current car controls
        car_controls2 = client.getCarControls()
        # Get the current steering
        curr_steer = car_controls2.steering

        # Add the new data to the car data array
        new_car_data = [time_diff, steer_setpoint, curr_steer]
        car_data = np.append(car_data, [new_car_data], axis=0)

        # Wait for 0.01 seconds
        time.sleep(0.01)

    # Stop the car
    car_controls.throttle = 0.0
    client.setCarControls(car_controls)

    total_len = car_data.shape[0]
    # Plot the steering setpoint and the actual steering over time
    fig, ax = plt.subplots(1, 1)
    ax.plot(car_data[:, 0], car_data[:, 1], '-b')
    ax.plot(car_data[:, 0], car_data[:, 2], '-r')
    fig.show()
    pass

