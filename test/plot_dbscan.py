import numpy as np
import pickle
from matplotlib import pyplot as plt
import time
from spline_utils import PathSpline


"""
plot_dbscan.py
--------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Loads DBSCAN data from a pickle file.
2. Iterates over the centroids of each cluster.
3. Plots the centroids of each cluster on a 2D plot.

Dependencies:
- numpy: Python library for numerical computations.
- pickle: Standard Python library for serializing and de-serializing Python object structures.
- matplotlib: Python library for creating static, animated, and interactive visualizations in Python.
- spline_utils: Custom module for spline utilities.

Usage:
Run this script to perform the tasks mentioned above. Ensure that the DBSCAN data is available in a pickle file.
"""

# Load DBSCAN data from a pickle file
with open('dbscan_session.pickle', 'rb') as handle:
    dbscan_data = pickle.load(handle)

# Extract all segments and centroids from the loaded data
all_segments = dbscan_data['all_segments']
all_centroids = dbscan_data['all_centroids']

# Delete the loaded data to free up memory
del all_segments
del dbscan_data

# Create a new figure and axis for the plot
fig, ax = plt.subplots(1, 1)

# Iterate over all centroids
for idx in range(len(all_centroids)):
    # If there are centroids for the current index
    if all_centroids[idx]:
        # Initialize empty arrays for the current segments and centroids
        curr_segments = np.ndarray(shape=(0, 3))
        curr_centroids = np.ndarray(shape=(0, 3))
        # Iterate over all frames for the current index
        for curr_frame in all_centroids[idx]:
            # debug_seg = all_segments[idx][curr_frame]
            # curr_segments = np.append(curr_segments, all_segments[idx][curr_frame], axis=0)
            # debug_cent = all_centroids[idx][curr_frame].reshape(1, 3)

            # Append the current frame's centroids to the array of current centroids
            curr_centroids = np.append(curr_centroids, all_centroids[idx][curr_frame].reshape(1, 3), axis=0)


        # ax.plot(curr_segments[:, 0], curr_segments[:, 1], '.b')
        # Plot the current centroids on the figure
        ax.plot(curr_centroids[:, 0], curr_centroids[:, 1], 'or')

        # Set grid, limits and show the plot
        ax.grid(True)
        plt.xlim([0, 20])
        plt.ylim([-5, 5])
        fig.show()
        a = 5

        # Clear the axis for the next plot
        ax.clear()

