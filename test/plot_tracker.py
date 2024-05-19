import numpy as np
import pickle
from matplotlib import pyplot as plt
import time
import tracker_utils
import spline_utils

"""
plot_tracker.py
---------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Loads tracking data from a pickle file.
2. Iterates over the tracked objects and separates them based on their color.
3. Calculates the average position of the left and right points.
4. Generates a spline using the average positions.
5. Plots the tracked objects and the generated spline on a 2D plot.

Dependencies:
- numpy: Python library for numerical computations.
- pickle: Standard Python library for serializing and de-serializing Python object structures.
- matplotlib: Python library for creating static, animated, and interactive visualizations in Python.
- tracker_utils: Custom module for tracking utilities.
- spline_utils: Custom module for spline utilities.

Usage:
Run this script to perform the tasks mentioned above. Ensure that the tracking data is available in a pickle file.
"""


# Load tracking data from a pickle file
with open('tracker_session.pickle', 'rb') as handle:
    tracker_data = pickle.load(handle)

problematic = 0
left_points = np.ndarray(shape=(0, 2))
right_points = np.ndarray(shape=(0, 2))
unknown_points = np.ndarray(shape=(0, 2))

# Iterate over the tracked objects
for tracked_obj in tracker_data['cones']:
    if tracked_obj.active:
        # Separate the tracked objects based on their color
        if tracked_obj.color == tracker_utils.ConeTracker.COLOR_BLUE:
            left_points = np.append(left_points, tracked_obj.position[0:2].reshape(1, 2), axis=0)
        elif tracked_obj.color == tracker_utils.ConeTracker.COLOR_YELLOW:
            right_points = np.append(right_points, tracked_obj.position[0:2].reshape(1, 2), axis=0)
        else:
            point = tracked_obj.position[0:2]
            left_dist = np.linalg.norm(left_points - point, axis=1)
            right_dist = np.linalg.norm(right_points - point, axis=1)
            # Assign the tracked object to the closest side
            if np.min(left_dist) < np.min(right_dist):
                left_points = np.append(left_points, point.reshape(1, 2), axis=0)
            else:
                right_points = np.append(right_points, point.reshape(1, 2), axis=0)
            unknown_points = np.append(unknown_points, tracked_obj.position[0:2].reshape(1, 2), axis=0)
    else:
        print('inactive cone at pos:', tracked_obj.position[0:2])

# for point in unknown_points:
#     left_dist = np.linalg.norm(left_points - point, axis=1)
#     right_dist = np.linalg.norm(right_points - point, axis=1)
#     if np.min(left_dist) < np.min(right_dist):
#         left_points = np.append(left_points, point.reshape(1, 2), axis=0)
#     else:
#         right_points = np.append(right_points, point.reshape(1, 2), axis=0)

pursuit_points = np.ndarray(shape=(0, 2))

# Iterate over the pursuit points
for tracked_obj in tracker_data['pursuit']:
    pursuit_points = np.append(pursuit_points, tracked_obj[:2].reshape(1, 2), axis=0)

# Calculate the average position of the left and right points
# assuming somewhat equal cone detections on each side!
min_length = min(left_points.shape[0], right_points.shape[0])
track_points = (left_points[:min_length, :] + right_points[:min_length, :]) / 2


# Generate a spline using the average positions
my_spline = spline_utils.PathSpline(track_points[::2, 0], track_points[::2, 1])
my_spline.generate_spline(amount=0.1, meters=True, smoothing=1)

# Create a new figure and axis for the plot
fig, ax = plt.subplots(1, 1)

# Plot the tracked objects and the generated spline on the figure
ax.plot(right_points[:, 0], right_points[:, 1], 'or')
ax.plot(left_points[:, 0], left_points[:, 1], 'ob')
# ax.plot(unknown_points[:, 0], unknown_points[:, 1], 'ok')
# ax.plot(track_points[:, 0], track_points[:, 1], 'ok')
ax.plot(my_spline.xi, my_spline.yi)
# ax.plot(pursuit_points[:, 0], pursuit_points[:, 1], 'o')

# Set grid and equal axis for the plot
ax.grid(True)
ax.axis('equal')

# Show the plot
fig.show()
a = 5
