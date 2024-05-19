from spline_utils import PathSpline
import numpy as np
import airsim
from scipy.spatial.transform import Rotation as Rot
import time
from matplotlib import pyplot as plt
from pidf_controller import PidfControl
import csv
import pickle


"""
vehicle_speed_control.py
------------------------
This script is part of a simulation project for controlling the speed of a vehicle.

It performs the following tasks:

1. Connects to the AirSim simulator.
2. Sets the initial pose of the vehicle.
3. Initializes a PIDF controller for speed control.
4. Runs a loop for a specified duration.
5. In each iteration of the loop, it calculates the desired speed setpoint based on the elapsed time.
6. It then uses the PIDF controller to calculate the throttle command to achieve the desired speed.
7. The throttle command is sent to the vehicle in the simulator.
8. The actual speed of the vehicle is recorded for each iteration.
9. After the duration has passed, it plots the desired speed and the actual speed over time.

Dependencies:
- airsim: Python library for the AirSim simulator.
- numpy: Python library for numerical computations.
- time: Standard Python library for time-related tasks.
- matplotlib: Python library for creating static, animated, and interactive visualizations in Python.
- pidf_controller: Custom module for PIDF control.
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

# Main function to run the speed control
def run():
    # Initialize the PIDF controller for speed control
    # Airsim is stupid, always spawns at zero. Must compensate using "playerstart" in unreal:
    speed_controller = PidfControl(0.1)
    # Set the PIDF parameters
    # speed_controller.set_pidf(0.275, 0.3, 0.0, 0.044)
    speed_controller.set_pidf(0.1, 0.0, 0.0, 0.044)
    # Set the extrema for the PIDF controller
    speed_controller.set_extrema(0.01, 0.01)
    # Set the alpha value for the PIDF controller
    speed_controller.alpha = 0.01
    # Initialize the inputs and outputs arrays
    inputs = np.array([], dtype=float)
    outputs = np.array([], dtype=float)

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
    car_data = np.ndarray(shape=(0, 2))
    control_data = np.ndarray(shape=(0, 8))

    try:
        # Set the duration for the speed control
        duration = 20
        # Initialize the time variables
        current_time = time.time()
        start_time = current_time
        last_iteration = current_time
        run_time = current_time - start_time
        idx = 0
        new_setpoint = 1.0

        # Run the speed control for the set duration
        while run_time <= duration:
            # Get the current time
            current_time = time.time()
            # Calculate the iteration time and the run time
            iteration_time = current_time - last_iteration
            run_time = current_time - start_time

            # Change the speed setpoint at certain times
            if iteration_time >= 0.1:
                if run_time > 5.0:
                    new_setpoint = 2.0
                if run_time > 10.0:
                    new_setpoint = 4.0
                if run_time > 15.0:
                    new_setpoint = 8.0

                # Get the current vehicle state
                car_state = client.getCarState()
                # Get the current vehicle speed
                curr_vel = car_state.speed
                # Calculate the throttle command using the PIDF controller
                throttle_command = speed_controller.velocity_control(new_setpoint, 0, curr_vel)

                # Set the throttle command
                car_controls.throttle = throttle_command
                # Apply the car controls
                client.setCarControls(car_controls)
                # Increment the index
                idx += 1
                # Append the new setpoint and the current speed to the inputs and outputs arrays
                inputs = np.append(inputs, new_setpoint)
                outputs = np.append(outputs, curr_vel)
                # Record the time of the last iteration
                last_iteration = time.time()

        # Stop the vehicle
        car_controls.throttle = 0.0
        client.setCarControls(car_controls)

        # Plot the desired speed and the actual speed over time
        timeline = np.linspace(0, duration, idx)
        fig, ax = plt.subplots(1, 1)
        ax.plot(timeline, inputs, '-r')
        ax.plot(timeline, outputs, '-b')
        # ax.plot(inputs, '-r')
        # ax.plot(outputs, '-b')
        ax.grid(True)
        fig.show()
        pass

    except KeyboardInterrupt:
        # Stop the vehicle in case of a keyboard interrupt
        car_controls.throttle = 0.0
        client.setCarControls(car_controls)
        time.sleep(1.0)

    finally:
        # Stop the vehicle after the speed control is done
        car_controls.throttle = 0.0
        client.setCarControls(car_controls)
        time.sleep(1.0)


if __name__ == '__main__':
    run()
