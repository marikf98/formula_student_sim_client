import numpy as np
import pickle
import csv
import os
import airsim
import pidf_controller
import spatial_utils
import discrete_plant_emulator
import time
from matplotlib import pyplot as plt

"""
control_analysis.py
-----------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Connects to the AirSim simulator and enables API control.
2. Initializes the vehicle's starting point.
3. Defines the speed controller.
4. Defines the desired speed.
5. Records the car's speed for a certain duration while trying to maintain the desired speed.
6. Stops the car.
7. Saves the control data to a CSV file.
8. Plots the control data.

Dependencies:
- numpy: Python library for numerical computations.
- pickle: Standard Python library for serializing and de-serializing Python object structures.
- csv: Standard Python library for reading and writing CSV files.
- os: Standard Python library for interacting with the operating system.
- airsim: Python library for the AirSim simulator.
- pidf_controller: Custom module for PIDF control.
- spatial_utils: Custom module for spatial utilities.
- discrete_plant_emulator: Custom module for emulating a discrete plant.
- time: Standard Python library for time-related tasks.
- matplotlib: Python library for creating static, animated, and interactive visualizations in Python.

Usage:
Run this script to start the control analysis. Ensure that the AirSim simulator is running and the AirSim client is properly set up.
"""

# Define the directory where the CSV files will be stored
csv_folder = os.path.join(os.getcwd(), 'csv')

# Create an AirSim client
client = airsim.CarClient()

# Create an object to control the car
car_controls = airsim.CarControls()

# Initialize arrays to store the step data and control data
step_data = np.ndarray(shape=(0, 3))
control_data = np.ndarray(shape=(0, 4))

# Flags to indicate the type of test to be performed
bump_test = False

step_response = False
# Perform a bump test
if bump_test:
    # Connect to the AirSim simulator and enable API control
    client.confirmConnection()
    client.enableApiControl(True)

    # Initialize vehicle starting point
    spatial_utils.set_airsim_pose(client, [40.0, -80.0], [90.0, 0, 0])
    time.sleep(1.0)

    # Define the low and high step values
    low_step = 0.1
    high_step = 0.5

    # Set the throttle to the low step value and record the car's speed for 6 seconds
    car_controls.throttle = low_step
    client.setCarControls(car_controls)
    start_time1 = time.perf_counter()
    last_iteration = start_time1
    while time.perf_counter() - start_time1 < 6.0:
        if time.perf_counter() - last_iteration > 0.01:
            car_state = client.getCarState()
            step_data = np.append(step_data, [[time.perf_counter() - start_time1, low_step, car_state.speed]], axis=0)
            last_iteration = time.perf_counter()


    # Set the throttle to the high step value and record the car's speed for 10 seconds
    car_controls.throttle = high_step
    client.setCarControls(car_controls)
    start_time2 = time.perf_counter()
    last_iteration = start_time2
    while time.perf_counter() - start_time2 < 10.0:
        if time.perf_counter() - last_iteration > 0.01:
            car_state = client.getCarState()
            step_data = np.append(step_data, [[time.perf_counter() - start_time1, high_step, car_state.speed]], axis=0)
            last_iteration = time.perf_counter()

    # Stop the car
    car_controls.throttle = 0.0
    client.setCarControls(car_controls)

    # Save the step data to a CSV file
    with open(os.path.join(csv_folder, 'bump_test_' + str(high_step) + '.csv'), 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['time', 'step_size', 'car_speed'])
        writer.writerows(step_data)

    # Plot the step data
    fig, ax = plt.subplots(1, 1)
    ax.grid(True)
    ax.axis('equal')

    ax.plot(step_data[:, 0], step_data[:, 1], color='blue')
    ax.plot(step_data[:, 0], step_data[:, 2], color='red')

    fig.show()
    plt.waitforbuttonpress()


# Perform a step response test
elif step_response:
    # Connect to the AirSim simulator and enable API control
    client.confirmConnection()
    client.enableApiControl(True)

    # Initialize vehicle starting point
    spatial_utils.set_airsim_pose(client, [40.0, -80.0], [90.0, 0, 0])
    time.sleep(1.0)

    # Define speed controller:
    speed_controller = pidf_controller.PidfControl(0.01)
    speed_controller.set_pidf(0.4, 0.4, 0.0, 0.0)
    speed_controller.set_extrema(min_setpoint=0.01, max_integral=1.0)
    # speed_controller.set_pidf(0.04, 0.04, 0.0, 0.043)
    # speed_controller.set_extrema(min_setpoint=0.01, max_integral=0.1)
    speed_controller.alpha = 0.01

    # Define the desired speed
    desired_speed = 7.0

    # Record the car's speed for 10 seconds while trying to maintain the desired speed
    start_time = time.perf_counter()
    last_iteration = start_time
    while time.perf_counter() - start_time < 10.0:
        if time.perf_counter() - last_iteration > 0.01:
            car_state = client.getCarState()
            throttle = speed_controller.velocity_control(desired_speed, 0, car_state.speed)
            throttle = np.clip(throttle, 0.0, 1.0)
            car_controls.throttle = throttle
            client.setCarControls(car_controls)
            timestamp = time.perf_counter() - start_time
            control_data = np.append(control_data, [[timestamp, desired_speed, car_state.speed, throttle]], axis=0)
            last_iteration = time.perf_counter()

    # Stop the car
    car_controls.throttle = 0.0
    client.setCarControls(car_controls)

    # Save the control data to a CSV file
    with open(os.path.join(csv_folder, 'controlled_step_response.csv'), 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['time', 'desired_speed', 'car_speed', 'throttle'])
        writer.writerows(control_data)

    # Plot the control data
    fig, ax = plt.subplots(1, 1)
    ax.grid(True)
    ax.axis('equal')

    ax.plot(control_data[:, 0], control_data[:, 1], color='blue')
    ax.plot(control_data[:, 0], control_data[:, 2], color='red')
    ax.plot(control_data[:, 0], control_data[:, 3], color='green')

    fig.show()
    plt.waitforbuttonpress()

# Perform a different type of test
else:
    # Define the duration and time step of the test
    duration = 0.1
    dt = 0.001

    # Initialize an array to store the plant data
    plant_data = np.ndarray(shape=(0,4), dtype=float)

    # Define the first steering emulator and controller
    steer_emulator1 = discrete_plant_emulator.DiscretePlant(dt, 10, 4)
    steer_controller1 = pidf_controller.PidfControl(dt)
    steer_controller1.set_pidf(1000.0, 0.0, 12.6, 0.0)
    steer_controller1.set_extrema(0.01, 1.0)
    steer_controller1.alpha = 0.01

    # Define the second steering emulator and controller
    steer_emulator2 = discrete_plant_emulator.DiscretePlant(dt, 10, 4)
    steer_controller2 = pidf_controller.PidfControl(dt)
    steer_controller2.set_pidf(1000.0, 0.0, 15.0, 0.0)
    steer_controller2.set_extrema(0.01, 1.0)
    steer_controller2.alpha = 0.01

    # Define the setpoint
    setpoint = 40.0

    # Initialize the outputs
    output1 = 0.0
    output2 = 0.0

    # Perform the test
    idx = 0

    while idx * dt < duration:

        compensated_signal1 = steer_controller1.position_control(setpoint, output1)
        output1 = steer_emulator1.iterate_step(compensated_signal1)

        compensated_signal2 = steer_controller2.position_control(setpoint, output2)
        output2 = steer_emulator2.iterate_step(compensated_signal2)

        idx += 1
        plant_data = np.append(plant_data, [[idx * dt, setpoint, output1, output2]], axis=0)

    # Save the plant data to a CSV file
    with open(os.path.join(csv_folder, 'compensated_discrete.csv'), 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['time', 'setpoint', 'output1', 'output2'])
        writer.writerows(plant_data)

    print('saved csv data')

    # Plot the plant data
    timeline = np.linspace(0, duration, idx)
    fig, ax = plt.subplots(1, 1)
    ax.plot(timeline, plant_data[:, 1], '-k')
    ax.plot(timeline, plant_data[:, 2], '-b')
    ax.plot(timeline, plant_data[:, 3], '-r')
    ax.grid(True)
    fig.show()
    a = 5
    plt.waitforbuttonpress()


