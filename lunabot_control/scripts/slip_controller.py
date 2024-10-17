# all meters(etc.) unless otherwise mentioned
from math import cos, pi, sin


# https://robotics.jpl.nasa.gov/media/documents/advanced_robotics_2006.pdf
class SlipController:
    
    def __init__(self, k_1: float = 0.003, k_2: float = 0.003, kv: float = 0, kp: float = 3, ki: float = 0, kd: float = 0,
                 ka: float = 0):
        self.k_1 = k_1
        self.k_2 = k_2
        self.kv = kv
        self.ka = ka
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0

    # Combined func to output linear and angular velocity for robot
    def update_vel(self, robot_vel_slip, heading_error, dt, target_accel=0, target_vel=0):
        return (self.vel_pid_update(robot_vel_slip, dt, target_accel, target_vel),
                self.vel_ang_slip_update(robot_vel_slip, heading_error, dt))

    # Slip controller from paper for angular velocity, takes in slip vector and heading diff between robot and lookahead point
    def vel_ang_slip_update(self, robot_vel_slip, heading_error, dt):
        return (self.k_1 * heading_error + self.k_2 * robot_vel_slip[1]) / dt

    # Generic PIDF controller for linear velocity correction
    def vel_pid_update(self, robot_vel_slip, dt, target_accel=0, target_vel=0):
        """_summary_

        Args:
            target_vel (_type_): _description_
            actual_vel (_type_): _description_
            dt (_type_): _description_
            target_accel (int, optional): _description_. Defaults to 0.

        Returns:
            _type_: _description_
        """
        error = robot_vel_slip[0]
        self.integral += error * dt
        derivative = (error - self.last_error) / dt

        self.last_error = error

        return (self.kv * target_vel) + (self.ka * target_accel) + (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)


# Use this to find the theoretical pose (ignores slip)
class WheelOdometry:
    pose = {'x': 0, 'y': 0, 'theta': 0, 'vx': 0, 'vy': 0, 'vtheta': 0}
    wheelRadius = .2
    driveBase = .75  # distance between center of wheels (left/right) meters
    lastTime = 0  # s

    def updatePose(self, newTime, wheelSpeeds):
        """Updates the pose based on the accumulated encoder ticks
        of the two wheels. See https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
        for details.
        """
        leftVelo = wheelSpeeds[0]  # rotations/s
        rightVelo = wheelSpeeds[1]

        deltaTime = newTime - self.lastTime  # s

        leftTravel = leftVelo * deltaTime * 2 * pi * self.wheelRadius  # rotations/s * s * m = m
        rightTravel = rightVelo * deltaTime * 2 * pi * self.wheelRadius

        deltaTravel = (rightTravel + leftTravel) / 2
        deltaTheta = (rightTravel - leftTravel) / self.driveBase

        if rightTravel == leftTravel:
            deltaX = leftTravel * cos(self.pose['theta'])
            deltaY = leftTravel * sin(self.pose['theta'])
        else:
            radius = deltaTravel / deltaTheta

            # Find the instantaneous center of curvature (ICC).
            iccX = self.pose['x'] - radius * sin(self.pose['theta'])
            iccY = self.pose['y'] + radius * cos(self.pose['theta'])

            deltaX = cos(deltaTheta) * (self.pose['x'] - iccX) \
                     - sin(deltaTheta) * (self.pose['y'] - iccY) \
                     + iccX - self.pose['x']

            deltaY = sin(deltaTheta) * (self.pose['x'] - iccX) \
                     + cos(deltaTheta) * (self.pose['y'] - iccY) \
                     + iccY - self.pose['y']

        self.pose["x"] += deltaX
        self.pose["y"] += deltaY
        self.pose["theta"] = (self.pose['theta'] + deltaTheta) % (2 * pi)
        self.pose["vx"] = deltaTravel / deltaTime if deltaTime > 0 else 0.
        self.pose["vy"] = 0
        self.pose["vtheta"] = deltaTheta / deltaTime if deltaTime > 0 else 0.

        self.lastTime = newTime

    def getPose(self):
        return self.pose

    def setPose(self, newPose):
        self.pose = newPose


def main():
    pass