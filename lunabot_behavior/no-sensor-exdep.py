import rospy
import time

from lunabot_msgs.msg import RobotEffort
import interrupts
from geometry_msgs.msg import Twist
#from lunabot_control.scripts.pid_controller import VelocityPIDController
#from lunabot_control.scripts.clamp_output import clamp_output
from pid_controller import VelocityPIDController 
from clamp_output import clamp_output  
import ascent

from std_msgs.msg import Int8
from homing_controller import HomingController
import sys


class ExcavateNoSensor:
    """
    A state used to autonomously excavate. Plunge lowers linear actuators while spinning the buckets,
    and excavate drives forwards/spins the buckets to try and meet a target depth of cut. 
    No sensors required!
    """

    def __init__(
        self,
        excavation_publisher: rospy.Publisher = None,
        lin_act_publisher: rospy.Publisher = None,
        cmd_vel_publisher: rospy.Publisher = None,
    ):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if excavation_publisher is None:
            self.excavation_publisher: rospy.Publisher = rospy.Publisher("/excavate", Int8, queue_size=1, latch=True)
            rospy.init_node("excavation_node")
        else:
            self.excavation_publisher: rospy.Publisher = excavation_publisher

        if lin_act_publisher is None:
            self.lin_act_publisher: rospy.Publisher = rospy.Publisher("/lin_act", Int8, queue_size=1, latch=True)
        else:
            self.lin_act_publisher: rospy.Publisher = lin_act_publisher

        if cmd_vel_publisher is None:
            self.cmd_vel_publisher: rospy.Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
        else:
            self.cmd_vel_publisher: rospy.Publisher = cmd_vel_publisher

        self.rate = rospy.Rate(10)

        self.lin_act_speed = -110
        self.excavate_speed = int(127 * 0.8)

        self.is_sim = rospy.get_param("is_sim")

    def excavate(self):
        self.plunge()

    def plunge(self):
        """
        Controls plunging (moving linear actuators down + spinning excavation)
        """

        time.sleep(0.1)  # Why is time.sleep() here. TODO investigate

        print("Excavation: Plunging")

        if self.is_sim:
            rospy.loginfo("Plunge: would plunge")
            time.sleep(3)
            return True

        excavation_message = Int8()
        lin_act_message = Int8()

        start_time = rospy.get_time()

        # Until the linear actuators reach the end (based on time), keep moving them down
        while rospy.get_time() - start_time < self.LOWERING_TIME:

            if interrupts.check_for_interrupts() != interrupts.Errors.FINE:
                return False

            excavation_message.data = self.excavate_speed

            lin_act_message.data = self.lin_act_speed

            self.lin_act_publisher.publish(lin_act_message)
            self.excavation_publisher.publish(excavation_message)

            self.rate.sleep()

        lin_act_message.data = 0
        self.lin_act_publisher.publish(lin_act_message)

        return True


class DepositionNoSensor:
    '''
    State to deposit collected regolith onto the berm by spinning the auger. No sensors required!
    '''

    def __init__(self, deposition_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if deposition_publisher is None:
            self.deposition_publisher = rospy.Publisher("/deposition", Int8, queue_size=1, latch=True)
            rospy.init_node('deposition_node')
        else:
            self.deposition_publisher = deposition_publisher

        self.rate = rospy.Rate(10)  # 10hz

        self.deposition_power = 127 # TODO change if needed

        self.DEPOSITION_TIME = 45.00

        self.is_sim = rospy.get_param("is_sim")

    def deposit(self):
        """
        Spin the conveyor until the bucket is empty.
        """

        if (self.is_sim):
            rospy.loginfo("Deposition: would deposit")
            time.sleep(2)
            return True

        time.sleep(0.1)

        deposition_msg = Int8()
        deposition_msg.data = self.deposition_power

        start_time = rospy.get_time()

        while True:
            self.deposition_publisher.publish(deposition_msg)
            
            if rospy.get_time() - start_time > self.DEPOSITION_TIME:
                break

            if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                return False

            self.rate.sleep()

        deposition_msg.data = 0
        self.deposition_publisher.publish(deposition_msg)

        return True

if __name__ == "__main__":
    # Excavate in place
    excavate_module = ExcavateNoSensor()
    excavate_module.excavate()

    # Raise linear actuators out of the way
    lin_act_publisher = rospy.Publisher("/lin_act", Int8, queue_size=1, latch=True)
    ascent_module = ascent.Ascent(lin_act_publisher)
    print("Exc: Raising")
    ascent_module.raise_linear_actuators()

    # Drive to the berm area
    start_time = rospy.get_time()
    DRIVE_TIME = 5.0
    cmd_vel = Twist()
    cmd_vel.linear.x = -0.3
    cmd_vel.angular.z = 0
    cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)

    time.sleep(0.1)
    # Drive for drive_time seconds
    while rospy.get_time() - start_time < DRIVE_TIME:
        cmd_vel_publisher.publish(cmd_vel)

    # When at the berm, stop
    cmd_vel.linear.x = 0
    cmd_vel_publisher.publish(cmd_vel)
    time.sleep(0.1)
    
    # Now at berm, deposit
    deposition = DepositionNoSensor()
    deposition.deposit()
