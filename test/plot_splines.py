import numpy as np
import pickle
from matplotlib import pyplot as plt
import time
from spline_utils import PathSpline

"""
plot_splines.py
---------------
This script is part of the Formula Racing Simulation for Ben Gurion University.

It performs the following tasks:

1. Defines the starting point and the origin of the spline.
2. Defines the x and y coordinates of the spline.
3. Generates the spline using the defined coordinates.
4. Plots the generated spline on a 2D plot.

Dependencies:
- numpy: Python library for numerical computations.
- pickle: Standard Python library for serializing and de-serializing Python object structures.
- matplotlib: Python library for creating static, animated, and interactive visualizations in Python.
- spline_utils: Custom module for spline utilities.

Usage:
Run this script to perform the tasks mentioned above.
"""

# with open('car_data.pickle', 'rb') as handle:
#     car_data = pickle.load(handle)
#
# with open('control_data.pickle', 'rb') as handle:
#     control_data = pickle.load(handle)

# Define the starting point and the origin of the spline
starting_x = 12
starting_y = 19
spline_origin_x = 12
spline_origin_y = 25
x_offset = spline_origin_x - starting_x
y_offset = spline_origin_y - starting_y

# Define the x and y coordinates of the spline
x = np.array([-2.00, 0.00, 0.00, 0.00, -5.00, -13.0, -21.0, -27.0, -32.0, -38.0, -47.0, -55.0, -53.0, -40.0, -25.0, -23.0, -37.0, -34.0, -20.0, -8.0])
y = np.array([6.0, -7.0, -19.0, -34.0, -46.0, -51.0, -54.0, -59.0, -68.0, -74.0, -75.0, -68.0, -54.0, -39.0, -23.0, -8.00, 6.00, 21.00, 23.00, 15.00])

# Adjust the coordinates based on the offset
x += x_offset
y += y_offset
y *= -1

# Generate the spline using the defined coordinates
spline_obj = PathSpline(x, y)
spline_obj.generate_spline(0.1, smoothing=1)

# Create a new figure and axis for the plot
fig, ax = plt.subplots(1, 1)

# Set grid and equal axis for the plot
ax.grid(True)
ax.axis('equal')

# Plot the generated spline on the figure
ax.plot(spline_obj.xi, spline_obj.yi, color='red', linewidth=0.2)

# Show the plot
fig.show()


# The following commented code is for plotting car data, which is not used in this script
# for curr_row in range(car_data.shape[0]):
#     ax.plot(car_data[:, 0], car_data[:, 1], '.b')
#     ax.plot(car_data[curr_row, 0], car_data[curr_row, 1], 'or')
#     dir_x = [car_data[curr_row, 0],
#              car_data[curr_row, 0] + car_data[curr_row, 3] * np.cos(car_data[curr_row, 2])]
#     dir_y = [car_data[curr_row, 1],
#              car_data[curr_row, 1] + car_data[curr_row, 3] * np.sin(car_data[curr_row, 2])]
#     ax.plot(dir_x, dir_y, 'or')
#     fig.show()
#     ax.cla()

# fig, ax = plt.subplots(1, 1)
# ax.plot(car_data[:, 3], '.b')
# fig.show()
# a=5
pass