import time

"""
time_test.py
------------
This script is part of a testing suite for a simulation project.

It performs the following tasks:

1. Sets a duration for the test.
2. Starts a timer.
3. Runs a loop for the duration of the test.
4. In each iteration of the loop, it calculates the time taken for the iteration.
5. Appends the iteration time to a list.
6. Prints the time taken for each iteration after the test duration has passed.

Dependencies:
- time: Standard Python library for time-related tasks.

Usage:
Run this script to perform the tasks mentioned above. This script is used for testing the time taken for each iteration of a loop.
"""

# Set the duration for the test
duration = 2.0

# Start the timer
start_time = time.time()
# Initialize the run time
run_time = 0.0

# Record the time of the last iteration
last_iteration = time.time()

# Initialize the list to store the time taken for each iteration
all_iterations = []


# Run the loop for the duration of the test
while run_time < duration:
    # Get the current time
    current_time = time.time()
    # Calculate the run time
    run_time = current_time - start_time

    # Calculate the time taken for the iteration
    iteration_time = current_time - last_iteration

    # Append the iteration time to the list
    all_iterations.append(iteration_time)

    # Record the time of the last iteration
    last_iteration = time.perf_counter()

    # Print the time taken for the iteration
    print(last_iteration - time.perf_counter())
    # time.sleep(0.001)
    # if iteration_time > 0.001:
    #     # print(iteration_time)
    #     all_iterations.append(iteration_time)
    #     last_iteration = time.time()

# Print the time taken for each iteration after the test duration has passed
print(all_iterations)
