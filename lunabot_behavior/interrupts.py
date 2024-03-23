import rospy
from enum import Enum, auto
from lunabot_msgs.msg import RobotErrors

#TODO: add catastrophic failure state? (manual control)
class Errors(Enum):
    FINE = auto()
    OVERCURRENT = auto()
    ROS_ENDED = auto()
    STUCK = auto()


class Interrupts:
    def errors_callback(self, msg: RobotErrors):
        self.robot_errors = msg

    def __init__(self):
        self.robot_errors = RobotErrors()

        rospy.Subscriber("/errors", RobotErrors, self.errors_callback)

    def main(self):
        if self.robot_errors.stuck:
            return Errors.STUCK
        if self.robot_errors.overcurrent:
            return Errors.OVERCURRENT
        if rospy.is_shutdown():
            return Errors.ROS_ENDED

        return Errors.FINE
    
interrupt_class = Interrupts()

def check_for_interrupts():
    # This is done so, in every other module, you can call interrupts.check_for_interrupts()
    # and not every module needs a copy of the class
    return interrupt_class.main()
