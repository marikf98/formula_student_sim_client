# Not based upon any imports
"""
pidf_controller.py
------------------
This script is part of a simulation project for controlling a vehicle's path.

It performs the following tasks:

1. Defines a class `PidfControl` for implementing the PIDF control algorithm.
2. The `PidfControl` class initializes with a sample time and has methods to set the PIDF coefficients and extrema.
3. The class has methods `compute_velocity` and `compute_integral` to compute the velocity and integral term respectively.
4. The class has methods `velocity_control` and `position_control` to perform velocity and position control respectively.

Dependencies:
- None

Usage:
Import this module to use the `PidfControl` class for controlling a vehicle's path. The PIDF control algorithm is used to calculate the control output based on the setpoint and the vehicle's current position and velocity.
"""

# Function to filter the value
def filter_value(current_val, previous_val, alpha):
    return previous_val * alpha + current_val * (1.0-alpha)

# Function to clamp the value between min and max
def clamp(my_value, min_value, max_value):
    return max(min(my_value, max_value), min_value)


# PIDF Control class
class PidfControl:
    def __init__(self, sample_time):
        # Initialize with sample time and other parameters
        self.kp = 1.0  # Proportional
        self.ki = 0.0  # Integral
        self.kd = 0.0  # Derivative
        self.kf = 0.0  # Feed-forward

        self.alpha = 0.5
        self.dt = sample_time

        self.pos_prev = 0.0
        self.vel_prev = 0.0
        self.filtered_vel = 0.0

        self.integral = 0.0
        self.max_integral = 1.0
        self.min_setpoint = 0.0

    # Method to set the PIDF coefficients
    def set_pidf(self, kp, ki, kd, kf):
        self.kp = kp  # Proportional
        self.ki = ki  # Integral
        self.kd = kd  # Derivative
        self.kf = kf  # Feed-forward

    # Method to set the extrema
    def set_extrema(self, min_setpoint, max_integral):
        self.max_integral = max_integral
        self.min_setpoint = min_setpoint

    # Method to compute the velocity
    # The measurement input is a position, hence it is differentiated before computing the error.
    # Units depend on caller function's use case.
    def compute_velocity(self, pos, vel=None):
        # In most cases we would want to supply position data and derive velocity from it.
        # But, in some cases we might have the velocity already available,
        # for example if we have a gyro attached to the shaft.
        if vel is None:
            raw_vel = (pos - self.pos_prev) / self.dt
            self.pos_prev = pos
        else:
            raw_vel = vel

        self.filtered_vel = filter_value(raw_vel, self.vel_prev, self.alpha)
        if abs(self.filtered_vel) < 0.001:  # Disable unnecessary computations at low values
            self.filtered_vel = 0

        self.vel_prev = self.filtered_vel

    # Method to compute the integral term
    def compute_integral(self, err):
        self.integral += err * self.ki
        self.integral = clamp(self.integral, -self.max_integral, self.max_integral)  # Assuming symmetrical behaviour

    # Method to perform velocity control
    def velocity_control(self, setpoint, pos, vel=None):
        self.compute_velocity(pos, vel)
        err = setpoint - self.filtered_vel
        self.compute_integral(err)
        # We want to disable the integral term if the setpoint is too low to prevent currents:
        if abs(setpoint) < self.min_setpoint:
            self.integral = 0
        output = err * self.kp + self.integral + self.kf * setpoint
        return output

    # Method to perform position control
    def position_control(self, setpoint, pos, vel=None):
        self.compute_velocity(pos, vel)
        err = setpoint - pos
        self.compute_integral(err)
        output = err * self.kp + self.integral - self.kd * self.filtered_vel
        return output
