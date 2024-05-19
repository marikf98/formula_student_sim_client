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
speed_bump_test.py
------------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Connects to the AirSim simulator.
2. Sets the initial pose of the vehicle.
3. Controls the vehicle to move with a certain throttle.
4. Collects the vehicle's speed data over time while it's moving.
5. Saves the collected data into a CSV file.
6. Calculates the average velocity for the given throttle.
7. Plots the speed data over time.

Dependencies:
- airsim: Python library for the AirSim simulator.
- numpy: Python library for numerical computations.
- time: Standard Python library for time-related tasks.
- csv: Standard Python library for reading and writing CSV files.
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

    # Define the starting point for the AirSim simulator
    # Airsim is stupid, always spawns at zero. Must compensate using "playerstart" in unreal:
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
    # Initialize the data arrays
    car_data = np.ndarray(shape=(0, 2))
    control_data = np.ndarray(shape=(0, 8))

    # Set the target throttle
    target_throttle = 0.5
    car_controls.throttle = target_throttle
    # Apply the car controls
    client.setCarControls(car_controls)

    # Start the timer
    start_time = time.time()
    # Collect data for 12 seconds
    while time.time() - start_time < 12.0:
        # Calculate the time difference
        time_diff = time.time() - start_time
        # Get the current car state
        car_state = client.getCarState()
        # Get the current car pose
        car_pose = client.simGetVehiclePose()
        # Calculate the current position
        curr_pos = [car_pose.position.x_val + starting_x, car_pose.position.y_val + starting_y]
        # Get the current velocity
        curr_vel = car_state.speed

        # Add the new data to the car data array
        new_car_data = [time_diff, curr_vel]
        car_data = np.append(car_data, [new_car_data], axis=0)

        # print('current timestamp: ', time_diff)
        # Wait for 0.01 seconds
        time.sleep(0.01)


    # Stop the car
    car_controls.throttle = 0.0
    client.setCarControls(car_controls)

    # Begin data processing:
    # Save the data to a CSV file
    with open('bump_test_torque_2000_step_' + str(target_throttle) + '.csv', 'w', newline='', encoding='utf-8') as f:
        # create the csv writer
        writer = csv.writer(f)
        writer.writerow(['time [s]', 'speed [m/s]'])
        writer.writerows(car_data)

    # Calculate the average velocity
    total_len = car_data.shape[0]
    print('average velocity for throttle:', target_throttle, 'is:', np.average(car_data[2*total_len//3:, 1]), 'm/s')
    # Plot the speed data over time
    fig, ax = plt.subplots(1, 1)
    ax.plot(car_data[:, 0], car_data[:, 1], '.b')
    fig.show()
    pass

