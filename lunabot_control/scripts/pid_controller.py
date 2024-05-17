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
        self.total_error = 0 #Reset I when changing goal
    
    def update(self, state, dt):
        error = self.set_setpoint - state
        self.total_error += error * dt
        error_diff = (error - self.prev_error) / dt
        self.prev_error = error
        return self.setpoint * self.kf + error * self.kp + self.total_error * self.ki + error_diff * self.kd

