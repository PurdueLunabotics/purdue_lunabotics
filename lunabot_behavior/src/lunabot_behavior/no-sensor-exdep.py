import rospy
import time

from geometry_msgs.msg import Twist
#from lunabot_control.scripts.pid_controller import VelocityPIDController
#from lunabot_control.scripts.clamp_output import clamp_output

from lunabot_behavior.excavate import ExcavationController
from lunabot_behavior.deposition import DepositionManager
from lunabot_behavior.linear_actuators import LinearActuatorManager

from std_msgs.msg import Int8
import sys

class NoSensorExdepManager:
    def __init__(self, excavation_publisher: rospy.Publisher = None, linear_actuator_publisher: rospy.Publisher = None, cmd_vel_publisher: rospy.Publisher = None, deposition_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """
        if excavation_publisher is None or linear_actuator_publisher is None or cmd_vel_publisher is None or deposition_publisher is None:
            rospy.init_node('no_sensor_exdep_node')

            self.excavation_publisher: rospy.Publisher = rospy.Publisher("/excavate", Int8, queue_size=1, latch=True)
            self.lin_act_publisher: rospy.Publisher = rospy.Publisher("/lin_act", Int8, queue_size=1, latch=True)
            self.deposition_publisher: rospy.Publisher = rospy.Publisher("/deposition", Int8, queue_size=1, latch=True)
            self.cmd_vel_publisher: rospy.Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
        else:
            self.excavation_publisher = excavation_publisher
            self.lin_act_publisher = linear_actuator_publisher
            self.deposition_publisher = deposition_publisher
            self.cmd_vel_publisher = cmd_vel_publisher

        self.excavation = ExcavationController(self.excavation_publisher, self.lin_act_publisher, self.cmd_vel_publisher)
        self.deposition = DepositionManager(self.deposition_publisher)
        self.linear_actuators = LinearActuatorManager(self.lin_act_publisher)

        self.DRIVE_TIME = 5.0  # in seconds, TODO choose value
        self.DRIVE_SPEED = -0.3  # in m/s, TODO choose value

if __name__ == "__main__":

    exdepController = NoSensorExdepManager()

    # Excavate in place
    exdepController.excavation.no_sensor_excavate()

    # Raise linear actuators out of the way
    exdepController.linear_actuators.raise_linear_actuators(use_current=False)

    # Drive to the berm area
    start_time = rospy.get_time()

    cmd_vel = Twist()
    cmd_vel.linear.x = exdepController.DRIVE_SPEED
    cmd_vel.angular.z = 0
    exdepController.cmd_vel_publisher.publish(cmd_vel)

    time.sleep(0.1)
    # Drive for <drive_time> seconds
    while rospy.get_time() - start_time < managerDRIVE_TIME:
        exdepController.cmd_vel_publisher.publish(cmd_vel)

    # When at the berm (by time), stop
    cmd_vel.linear.x = 0
    exdepController.cmd_vel_publisher.publish(cmd_vel)
    time.sleep(0.1)
    
    # Now at berm, deposit
    exdepController.deposition.no_sensor_deposit()
