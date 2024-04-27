import rospy
import time

from lunabot_msgs.msg import RobotEffort, RobotSensors
import interrupts
#from lunabot_control.scripts.pid_controller import VelocityPIDController
#from lunabot_control.scripts.clamp_output import clamp_output
from pid_controller import VelocityPIDController 
from clamp_output import clamp_output  

from std_msgs.msg import Int8


class Excavate:
    """
    A state used to autonomously excavate. Plunge lowers linear actuators while spinning the buckets,
    and excavate drives forwards/spins the buckets to try and meet a target depth of cut.
    """

    def sensors_callback(self, msg: RobotSensors):
        self.robot_sensors = msg

    def __init__(
        self,
        excavation_publisher: rospy.Publisher = None,
        lin_act_publisher: rospy.Publisher = None,
        left_drive_publisher: rospy.Publisher = None,
        right_drive_publisher: rospy.Publisher = None,
    ):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if excavation_publisher is None:
            self.excavation_publisher: rospy.Publisher = rospy.Publisher(
                "/excavate", Int8, queue_size=1, latch=True
            )
            rospy.init_node("plunge_node")
        else:
            self.excavation_publisher: rospy.Publisher = excavation_publisher

        if lin_act_publisher is None:
            self.lin_act_publisher: rospy.Publisher = rospy.Publisher(
                "/lin_act", Int8, queue_size=1, latch=True
            )
        else:
            self.lin_act_publisher: rospy.Publisher = lin_act_publisher

        if left_drive_publisher is None:
            self.left_drive_publisher: rospy.Publisher = rospy.Publisher(
                "/left_drive", Int8, queue_size=1, latch=True
            )
        else:
            self.left_drive_publisher: rospy.Publisher = left_drive_publisher

        if right_drive_publisher is None:
            self.right_drive_publisher: rospy.Publisher = rospy.Publisher(
                "/right_drive", Int8, queue_size=1, latch=True
            )
        else:
            self.right_drive_publisher: rospy.Publisher = right_drive_publisher

        self.robot_sensors = RobotSensors()

        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.rate = rospy.Rate(10)

        # 90 percent of max speed
        TARGET_EXCAVATION_VELOCITY = 127 * 0.9

        self.excavation_pid_controller = VelocityPIDController(
            TARGET_EXCAVATION_VELOCITY, 1, 0, 0, 127
        )  # TODO find values

        self.left_drivetrain_pid_controller = VelocityPIDController(
            1, 1, 0, 0, 127
        )  # TODO find values
        self.right_drivetrain_pid_controller = VelocityPIDController(
            1, 1, 0, 0, 127
        )  # TODO find values

        # Constants (in meters)
        self.TARGET_DEPTH_OF_CUT = 0.003175  # Currently set to 1/8 inch, TODO tune this

        self.BUCKET_RADIUS = 0.0948
        self.BUCKET_SPACING = 0.0853

        self.LOWERING_TIME = 30  # seconds
        self.TRENCHING_TIME = 12  # seconds

        self.load_cell_weight_threshold = 0  # TODO find value
        self.max_lin_act_vel = (
            0.00688405797
        )  # In meters/s, the speed of the linear actuators at the max power
        self.lin_act_max_power = 110
        # from experiment - 19 cm / 27.6 seconds

        self.lin_act_curr_threshold = 10  # TODO find value

        self.excavation_current_threshold = 25  # Amps; TODO find/confirm value

        self.is_sim = rospy.get_param("is_sim")

    def excavate(self):
        self.plunge()
        self.trench()

    def plunge(self):
        """
        Controls plunging (moving linear actuators down + spinning excavation)
        """

        time.sleep(0.1)  # Why is time.sleep() here. TODO investigate

        if self.is_sim:
            rospy.loginfo("Plunge: would plunge")
            time.sleep(3)
            return True

        excavation_message = Int8()
        lin_act_message = Int8()

        excavation_ang = (
            self.robot_sensors.exc_ang
        )  # TODO have firmware give excavation angle directly?
        start_time = rospy.get_time()
        current_time = start_time

        # Until the linear actuators reach the end (based on time), keep moving them down
        while rospy.get_time() - start_time < self.LOWERING_TIME:

            if interrupts.check_for_interrupts() != interrupts.Errors.FINE:
                return False

            new_time = rospy.get_time()
            new_excavation_ang = self.robot_sensors.exc_ang

            dt = new_time - current_time

            # Set excavation based on the PID controller
            excavation_velocity = (
                new_excavation_ang - excavation_ang
            ) / dt  # TODO check calculations are good

            excavation_control = self.excavation_pid_controller.update(
                excavation_velocity, dt
            )
            excavation_message.data = clamp_output(excavation_control)

            # Set the target linear actuator velocity to target the depth of cut
            target_actuator_velocity = (
                self.TARGET_DEPTH_OF_CUT
                * excavation_velocity
                * self.BUCKET_RADIUS
                / self.BUCKET_SPACING
            )

            lin_act_message.data = clamp_output(
                target_actuator_velocity / self.max_lin_act_vel * self.lin_act_max_power
            )  # No encoders so cannot do PID, estimating (will slighly underestimate on lower voltage)

            self.lin_act_publisher.publish(lin_act_message)
            self.excavation_publisher.publish(excavation_message)

            # Check for excavation getting stuck (high current)
            if self.robot_sensors.exc_curr > self.excavation_current_threshold:
                self.spin_excavation_backwards()

            current_time = new_time
            excavation_ang = new_excavation_ang

            self.rate.sleep()

        lin_act_message.data = 0
        self.lin_act_publisher.publish(lin_act_message)

        return True

    def trench(self):
        """
        Controls trenching (spinning excavation + driving forward)
        """

        time.sleep(0.1)  # Why is time.sleep() here. TODO investigate

        if self.is_sim:
            rospy.loginfo("Trenching: would trench")
            time.sleep(3)
            return True

        excavation_message = Int8()
        right_drive_message = Int8()
        left_drive_message = Int8()

        excavation_ang = (
            self.robot_sensors.exc_ang
        )  # TODO have firmware give excavation angle directly?
        current_time = rospy.get_time()
        start_time = current_time

        """
        TODO: no load cells yet

        load_cell_weight = self.robot_sensors.load_cell_weights[0] + self.robot_sensors.load_cell_weights[1]

        original while condition: load_cell_weight < self.load_cell_weight_threshold
        """

        # TODO add logic for stopping if obstacles exist (both rocks and craters)

        # Until the set amount of time, keep moving the robot forward and spinning excavation
        while rospy.get_time() - start_time < self.TRENCHING_TIME:

            if interrupts.check_for_interrupts() != interrupts.Errors.FINE:
                return False

            new_time = rospy.get_time()
            new_excavation_ang = self.robot_sensors.exc_ang

            dt = new_time - current_time

            excavation_velocity = (
                new_excavation_ang - excavation_ang
            ) / dt  # TODO check calculations are good

            excavation_control = self.excavation_pid_controller.update(
                excavation_velocity, dt
            )
            excavation_message.data = clamp_output(excavation_control)

            target_linear_vel = (
                self.TARGET_DEPTH_OF_CUT
                * excavation_velocity
                * self.BUCKET_RADIUS
                / self.BUCKET_SPACING
            )

            self.left_drivetrain_pid_controller.set_setpoint(target_linear_vel)
            left_drivetrain_ctrl = self.left_drivetrain_pid_controller.update(
                self.robot_sensors.drive_left_vel, dt
            )
            left_drive_message.data = clamp_output(left_drivetrain_ctrl)

            self.right_drivetrain_pid_controller.set_setpoint(target_linear_vel)
            right_drivetrain_ctrl = self.right_drivetrain_pid_controller.update(
                self.robot_sensors.drive_right_vel, dt
            )
            right_drive_message.data = clamp_output(right_drivetrain_ctrl)

            self.excavation_publisher.publish(excavation_message)
            self.left_drive_publisher.publish(left_drive_message)
            self.right_drive_publisher.publish(right_drive_message)

            # Check for excavation getting stuck (high current)
            if self.robot_sensors.exc_curr > self.excavation_current_threshold:
                self.spin_excavation_backwards()

            current_time = new_time
            excavation_ang = new_excavation_ang

            load_cell_weight = (
                self.robot_sensors.load_cell_weights[0]
                + self.robot_sensors.load_cell_weights[1]
            )

            self.rate.sleep()

        excavation_message.data = 0
        left_drive_message.data = 0
        right_drive_message.data = 0

        self.excavation_publisher.publish(excavation_message)
        self.left_drive_publisher.publish(left_drive_message)
        self.right_drive_publisher.publish(right_drive_message)

        return True

    def spin_excavation_backwards(self):
        """
        Spins excavation backwards for 1 second. Used to unstick rocks from buckets (if excavation gets high current)
        """

        # 90% of max speed, backwards
        EXCAVATION_SPEED = int(-127 * 0.9)

        excavation_message = Int8()
        excavation_message.data = EXCAVATION_SPEED

        self.excavation_publisher.publish(excavation_message)

        rospy.sleep(1)

        excavation_message.data = 0
        self.excavation_publisher.publish(excavation_message)


if __name__ == "__main__":
    excavate_module = Excavate()
    excavate_module.excavate()
