import numpy as np


class VelocityPIDController:
    """
    Velocity PID Controller
    Takes in setpoint, PID values, and feedforward term
    Feedforward term should be max speed in control space / max speed in sensor space (127 / max speed of motor for most cases)
    """

    def __init__(self, setpoint, kp, ki, kd, kf):
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf
        self.total_error = 0
        self.prev_error = 0

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint
        self.total_error = 0  # Reset I when changing goal

    def update(self, state, dt):
        error = self.set_setpoint - state
        self.total_error += error * dt
        error_diff = (error - self.prev_error) / dt
        self.prev_error = error
        return self.setpoint * self.kf + error * self.kp + self.total_error * self.ki + error_diff * self.kd


class PIDController:
    """
    PID Controller
    Generic controller that gives PID-computed value based on setpoint and current position.
    """

    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0, max_output: float = 1., min_output: float = 0.):
        """
        Initialize PID controller with setpoint and PID values

        :param kp: Proportional gain
        :param ki: Integral gain
        :param kd: Derivative gain
        :param setpoint: Setpoint
        """
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.total_error = 0
        self.prev_error = 0
        self.max_output = max_output
        self.min_output = min_output
        self.tolerance = 0

    def set_max_value(self, max_value: float):
        """
        Set maximum value for PID controller

        :param max_value: Maximum value
        """
        self.max_output = max_value

    def set_tolerance(self, min_output: float):
        """
        Set minimum output value for PID controller.

        :param min_output: minimum output
        """
        self.min_output = min_output

    def set_setpoint(self, setpoint: float):
        """
        Set new setpoint for PID controller

        :param setpoint: New setpoint"""
        self.prev_setpoint = self.setpoint
        self.setpoint = setpoint
        if (self.prev_setpoint != self.setpoint):
            self.total_error = 0  # reset I when changing goal

    def calculate(self, state: float, dt: float, setpoint: float = 0.) -> float:
        """
        Calculate PID value based on current state and time step

        :param state: Current state
        :param dt: Time step
        :return: PID value
        """
        self.setpoint = setpoint

        error = self.setpoint - state
        self.total_error += error * dt
        error_diff = (error - self.prev_error) / dt
        self.prev_error = error
        output = np.clip(error * self.kp + self.total_error * self.ki +
                         error_diff * self.kd, -self.max_output, self.max_output)

        if (abs(output) < self.min_output):
            output = 0

        return output
