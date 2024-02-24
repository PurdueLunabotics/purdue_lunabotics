from enum import Enum, auto

from lunabot_msgs.msg import RobotErrors


class Errors(Enum):
    OVERCURRENT = auto()
    MAP_CHANGE = auto()
    ROS_ENDED = auto()
    STUCK = auto()
    FINE = auto()


class Interrupts:
    def __init__(self, robot_errors: RobotErrors = None):
        self.robot_errors: RobotErrors = RobotErrors()

    def main():
        if robot_errors.stuck:
            return Errors.STUCK
        if robot_errors.overcurrent:
            return Errors.OVERCURRENT
        if rospy.is_shutdown():
            return Errors.ROS_ENDED

        return Errors.FINE

    def errors_callback(self, msg: RobotErrors):
        self.robot_errors = msg
