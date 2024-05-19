import cone_mapping
import path_following
import airsim
import spline_utils
import path_control
import os
import pickle
import csv
import time


"""
main_in_a_loop.py
-----------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It uses the Airsim client to perform the following tasks:

1. Establishes a connection with the Airsim client and enables API control.
2. Initiates the cone mapping process, which involves detecting cones and spline points and returning their locations.
3. Stops the vehicle and generates a path to follow using the detected cones and spline points.
4. Follows the generated path using Stanley's method.
5. Stops the vehicle once the path following process is complete.
6. Repeats the above steps for a specified number of runs (10 in this case).
7. After each run, it saves the mapping data and the following data to a specified directory.

Dependencies:
- cone_mapping: Module for detecting cones and spline points.
- path_following: Module for following the generated path.
- airsim: AirSim Python client library.
- spline_utils: Utility module for generating and working with splines.
- path_control: Module for controlling the path following process.
- os: Standard Python library for interacting with the operating system.
- pickle: Standard Python library for serializing and de-serializing Python object structures.
- csv: Standard Python library for reading and writing CSV files.
- time: Standard Python library for time-related tasks.

Usage:
Run this script to start the simulation. Ensure that the Unreal Engine simulation is running and the Airsim client is properly set up.
"""

if __name__ == '__main__':

    # Create an airsim client instance:
    steering_procedure_manager = path_control.SteeringProcManager() # Create a Steering Procedure Manager instance
    airsim_client = airsim.CarClient() # Create an AirSim Car Client instance
    airsim_client.confirmConnection() # Confirm the connection to the AirSim client
    airsim_client.enableApiControl(True) # Enable API control for the AirSim client

    for run_idx in range(10): # Loop for a specified number of runs
        data_dest = os.path.join(os.getcwd(), 'recordings', 'recording' + str(run_idx)) # Define the destination directory for the recordings
        os.makedirs(data_dest) # Create the recording directory if it doesn't exist

        # Detect the cones and spline points, and return their location:
        print('Starting on-the-fly cone mapping with constant speed and steering procedure.')
        mapping_data, pursuit_points = cone_mapping.mapping_loop(airsim_client) # Start the cone mapping process
        print('Mapping complete!')

        # Stop until spline generation is complete:
        print('Stopping vehicle and generating a path to follow...')
        car_controls = airsim_client.getCarControls() # Get the current car controls
        car_controls.throttle = 0.0 # Set the throttle to 0 to stop the car
        airsim_client.setCarControls(car_controls) # Apply the updated car controls

        # Save mapping data
        tracked_objects = {'cones': mapping_data, 'pursuit': pursuit_points} # Create a dictionary of the tracked objects
        with open(os.path.join(data_dest, 'mapping_session.pickle'), 'wb') as pickle_file: # Open a pickle file to save the tracked objects
            pickle.dump(tracked_objects, pickle_file) # Save the tracked objects to the pickle file
        print('pickle saved')

        # Arrange the points and generate a path spline:
        track_points = spline_utils.generate_path_points(mapping_data) # Generate path points from the mapping data
        spline_obj = spline_utils.PathSpline(track_points[::2, 0], track_points[::2, 1]) # Create a Path Spline object
        spline_obj.generate_spline(amount=0.1, meters=True, smoothing=1) # Generate the spline for the path
        print('Done!')

        # Follow the spline using Stanley's method:
        print('Starting variable speed spline following procedure.')
        pickling_objects, car_data = path_following.following_loop(airsim_client, spline_obj) # Start the path following process
        print('Full process complete! stopping vehicle.')

        # Done! stop vehicle:
        car_controls = airsim_client.getCarControls() # Get the current car controls
        car_controls.throttle = 0.0 # Set the throttle to 0 to stop the car
        car_controls.brake = 1.0 # Apply the brake to ensure the car is stopped
        airsim_client.setCarControls(car_controls) # Apply the updated car controls
        steering_procedure_manager.terminate_steering_procedure() # Terminate the steering procedure

        # Save following data
        with open(os.path.join(data_dest, 'following_session.pickle'), 'wb') as pickle_file: # Open a pickle file to save the following data
            pickle.dump(pickling_objects, pickle_file)  # Save the following data to the pickle file
        print('saved pickle data')
        with open(os.path.join(data_dest, 'car_data.csv'), 'w', newline='') as csv_file: # Open a CSV file to save the car data
            writer = csv.writer(csv_file) # Create a CSV writer
            writer.writerow(['x', 'y', 'heading', 'v_desired', 'v_delivered',
                             's_desired', 's_delivered', 'throttle']) # Write the header row to the CSV file
            writer.writerows(car_data) # Write the car data to the CSV file
        print('saved csv data')

