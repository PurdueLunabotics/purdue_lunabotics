import rospy
import time
import sys

from lunabot_msgs.msg import RobotEffort, RobotSensors
from geometry_msgs.msg import Twist

from lunabot_control.pid_controller import VelocityPIDController
from lunabot_control.clamp_output import clamp_output
from lunabot_behavior.linear_actuators import LinearActuatorManager

from std_msgs.msg import Int8


class ExcavationController:
    """
    Used to autonomously excavate. Plunge lowers linear actuators while spinning the buckets,
    and excavate drives forwards/spins the buckets to try and meet a target depth of cut.
    """

    def sensors_callback(self, msg: RobotSensors):
        self.robot_sensors = msg

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

        if excavation_publisher is None or lin_act_publisher is None or cmd_vel_publisher is None:
            self.excavation_publisher: rospy.Publisher = rospy.Publisher("/excavate", Int8, queue_size=1, latch=True)
            self.lin_act_publisher: rospy.Publisher = rospy.Publisher("/lin_act", Int8, queue_size=1, latch=True)
            self.cmd_vel_publisher: rospy.Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
            rospy.init_node("excavation_node")
        else:
            self.excavation_publisher: rospy.Publisher = excavation_publisher
            self.lin_act_publisher: rospy.Publisher = lin_act_publisher
            self.cmd_vel_publisher: rospy.Publisher = cmd_vel_publisher

        self.robot_sensors = RobotSensors()

        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.rate = rospy.Rate(10)

        # 90 percent of max speed
        TARGET_EXCAVATION_VELOCITY = 8 * 0.9   # TODO find max speed, and what unit it's in

        self.excavation_pid_controller = VelocityPIDController(TARGET_EXCAVATION_VELOCITY, 1, 0, 0, 127)  # TODO pick feed-forward value from new control space

        # Constants (in meters)
        self.TARGET_DEPTH_OF_CUT = 0.005  # (.5 cm)
        self.BUCKET_RADIUS = 0.0948
        self.BUCKET_SPACING = 0.0853

        self.LOWERING_TIME = 30  # seconds
        self.TRENCHING_TIME = 5  # seconds

        self.LOAD_CELL_WEIGHT_THRESHOLD = 1  # In kg, TODO find value
        self.MAX_LIN_ACT_VEL = 0.00688405797  # In meters/s, the speed of the linear actuators at the max power (from experiment - 19 cm / 27.6 seconds)
        self.LIN_ACT_MAX_POWER = 110

        self.LIN_ACT_CURR_THRESHOLD = 10  # Amps; TODO find value

        self.EXCAVATION_CURR_THRESHOLD = 25  # Amps; TODO find/confirm value

        self.is_sim = rospy.get_param("is_sim")

        self.exc_failure_counter = 0

    def excavate(self):
        self.plunge()
        self.trench()

    def plunge(self):
        """
        Controls plunging (moving linear actuators down + spinning excavation)
        """

        time.sleep(0.1)  

        print("Excavation: Plunging")

        if self.is_sim:
            rospy.loginfo("Plunge: would plunge")
            time.sleep(3)
            return

        excavation_message = Int8()
        lin_act_message = Int8()

        start_time = rospy.get_time()
        current_time = start_time

        # Until the linear actuators reach the end (based on time), keep moving them down
        while rospy.get_time() - start_time < self.LOWERING_TIME:

            new_time = rospy.get_time()

            dt = new_time - current_time

            # Set excavation based on the PID controller
            excavation_velocity = self.robot_sensors.exc_vel
        

            excavation_control = self.excavation_pid_controller.update(excavation_velocity, dt)
            excavation_message.data = excavation_control  #TODO clip this to a maximum

            # Set the target linear actuator velocity to target the depth of cut
            target_actuator_velocity = (
                self.TARGET_DEPTH_OF_CUT
                * excavation_velocity
                * self.BUCKET_RADIUS
                / self.BUCKET_SPACING
            )

            lin_act_message.data = clamp_output(-target_actuator_velocity / self.MAX_LIN_ACT_VEL * self.LIN_ACT_MAX_POWER)  # No encoders on actuators so cannot do PID

            self.lin_act_publisher.publish(lin_act_message)
            self.excavation_publisher.publish(excavation_message)

            # Check for excavation getting stuck (high current)
            if self.robot_sensors.exc_curr > self.EXCAVATION_CURR_THRESHOLD:
                self.exc_failure_counter += 1
                self.spin_excavation_backwards()

            if (self.exc_failure_counter >= 7):
                break

            current_time = new_time

            self.rate.sleep()

        lin_act_message.data = 0
        self.lin_act_publisher.publish(lin_act_message)

    def no_sensor_plunge(self):
        """
        Plunge for when no sensors are running. Just lowers linear actuators.
        """

        time.sleep(0.1)  

        print("Excavation: Plunging")

        if self.is_sim:
            rospy.loginfo("Plunge: would plunge")
            time.sleep(3)
            return

        excavation_message = Int8()
        lin_act_message = Int8()

        start_time = rospy.get_time()
        current_time = start_time

        # Until the linear actuators reach the end (based on time), keep moving them down
        while rospy.get_time() - start_time < self.LOWERING_TIME:

            lin_act_message.data = self.LIN_ACT_MAX_POWER
            excavation_message.data = 1  # TODO find max exc speed

            self.lin_act_publisher.publish(lin_act_message)
            self.excavation_publisher.publish(excavation_message)

            self.rate.sleep()

        lin_act_message.data = 0
        self.lin_act_publisher.publish(lin_act_message)

        # also stop excavation because we don't go into trenching
        excavation_message.data = 0
        self.excavation_publisher.publish(excavation_message)


    def trench(self):
        """
        Controls trenching (spinning excavation + driving forward)
        """

        time.sleep(0.1)

        print("Excavation: Trenching")

        if self.is_sim:
            rospy.loginfo("Trenching: would trench")
            time.sleep(3)
            return

        excavation_message = Int8()
        cmd_vel_message = Twist()

        current_time = rospy.get_time()
        start_time = current_time

        load_cell_weight = self.robot_sensors.load_cell_weights[0] + self.robot_sensors.load_cell_weights[1]

        # maybe TODO add logic for stopping if obstacles exist (both rocks and craters)

        # Until the set amount of time and while the robot is not yet full, keep moving the robot forward and spinning excavation
        while rospy.get_time() - start_time < self.TRENCHING_TIME and load_cell_weight < self.LOAD_CELL_WEIGHT_THRESHOLD:

            new_time = rospy.get_time()
            new_excavation_ang = self.robot_sensors.exc_ang

            dt = new_time - current_time

            excavation_velocity = self.robot_sensors.exc_vel

            excavation_control = self.excavation_pid_controller.update(excavation_velocity, dt)

            excavation_message.data = excavation_control # TODO clip this to a maximum

            self.excavation_publisher.publish(excavation_message)

            target_linear_vel = (
                self.TARGET_DEPTH_OF_CUT
                * excavation_velocity
                * self.BUCKET_RADIUS
                / self.BUCKET_SPACING
            )

            cmd_vel_message.linear.x = target_linear_vel
            cmd_vel_message.angular.z = 0

            self.cmd_vel_publisher.publish(cmd_vel_message)

            # Check for excavation getting stuck (high current)
            if self.robot_sensors.exc_curr > self.EXCAVATION_CURR_THRESHOLD:
                self.exc_failure_counter += 1
                self.spin_excavation_backwards()

            if (self.exc_failure_counter >= 7):
                break

            current_time = new_time

            load_cell_weight = (
               self.robot_sensors.load_cell_weights[0]
               + self.robot_sensors.load_cell_weights[1]
            )

            self.rate.sleep()

        excavation_message.data = 0
        cmd_vel_message.linear.x = 0
        cmd_vel_message.angular.z = 0

        self.excavation_publisher.publish(excavation_message)
        
        self.cmd_vel_publisher.publish(cmd_vel_message)

    def no_sensor_excavate(self):
        """
        Excavation for when no sensors are running. Just plunges.
        """
        self.no_sensor_plunge()


    def spin_excavation_backwards(self):
        """
        Stop excavation and spin excavation backwards for 2 seconds. Used to unstick rocks from buckets (if excavation gets high current)
        """

        print("Excavation: spinning backwards (", self.exc_failure_counter, ")")

        # 90% of max speed, backwards
        EXCAVATION_SPEED = int(-127 * 0.9)  #TODO find max speed

        excavation_message = Int8()
        excavation_message.data = 0
        self.excavation_publisher.publish(excavation_message)
        rospy.sleep(1)


        excavation_message.data = EXCAVATION_SPEED

        self.excavation_publisher.publish(excavation_message)

        rospy.sleep(2)

        excavation_message.data = 0
        self.excavation_publisher.publish(excavation_message)


if __name__ == "__main__":
    excavate_module = ExcavationController()

    if len(sys.argv) > 1 and (sys.argv[1] == "-n" or sys.argv[1] == "-no-sensor"):
        excavate_module.no_sensor_plunge()
    else:
        excavate_module.excavate()

    linear_actuators = LinearActuatorManager()
    print("Exc: Raising")
    linear_actuators.raise_linear_actuators()