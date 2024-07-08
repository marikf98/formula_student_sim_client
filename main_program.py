import cone_mapping
import path_following
import airsim
import spline_utils
import path_control

"""
main_program.py
---------------
This script is the main entry point for the Formula Racing Simulation for Ben Gurion University.

It uses the Airsim client to connect to the Unreal Engine simulation and perform the following tasks:

1. Establishes a connection with the Airsim client and enables API control.
2. Initiates the cone mapping process, which involves detecting cones and spline points and returning their locations.
3. Stops the vehicle and generates a path to follow using the detected cones and spline points.
4. Follows the generated path using Stanley's method.
5. Stops the vehicle once the path following process is complete.

Dependencies:
- cone_mapping: Module for detecting cones and spline points.
- path_following: Module for following the generated path.
- airsim: AirSim Python client library.
- spline_utils: Utility module for generating and working with splines.
- path_control: Module for controlling the path following process.

Usage:
Run this script to start the simulation. Ensure that the Unreal Engine simulation is running and the Airsim client is properly set up.
"""
if __name__ == '__main__':
    # Create an airsim client instance:
    steering_procedure_manager = path_control.SteeringProcManager()  # Create a Steering Procedure Manager instance
    airsim_client = airsim.CarClient()  # Create an AirSim Car Client instance
    airsim_client.confirmConnection()  # Confirm the connection to the AirSim client
    airsim_client.enableApiControl(True)  # Enable API control for the AirSim client

    # Detect the cones and spline points, and return their location:
    print('Starting on-the-fly cone mapping with constant speed and steering procedure.')
    mapping_data, pursuit_points = cone_mapping.mapping_loop(airsim_client)  # Start the cone mapping process
    print('Mapping complete!')

    # Stop until spline generation is complete:
    print('Stopping vehicle and generating a path to follow...')
    car_controls = airsim_client.getCarControls()  # Get the current car controls
    car_controls.throttle = 0.0  # Set the throttle to 0 to stop the car
    airsim_client.setCarControls(car_controls)  # Apply the updated car controls

    # Arrange the points and generate a path spline:
    track_points = spline_utils.generate_path_points(mapping_data)  # Generate path points from the mapping data
    spline_obj = spline_utils.PathSpline(track_points[::2, 0], track_points[::2, 1])  # Create a Path Spline object
    spline_obj.generate_spline(amount=0.1, meters=True, smoothing=1)  # Generate the spline for the path
    print('Done!')

    # Follow the spline using Stanley's method:
    print('Starting variable speed spline following procedure.')
    path_following.following_loop(airsim_client, spline_obj)  # Start the path following process
    print('Full process complete! stopping vehicle.')

    # Done! stop vehicle:
    car_controls = airsim_client.getCarControls()  # Get the current car controls
    car_controls.throttle = 0.0  # Set the throttle to 0 to stop the car
    car_controls.brake = 1.0  # Apply the brake to ensure the car is stopped
    airsim_client.setCarControls(car_controls)  # Apply the updated car controls
    steering_procedure_manager.terminate_steering_procedure()  # Terminate the steering procedure
