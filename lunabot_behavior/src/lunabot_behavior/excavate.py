#!/usr/bin/env python3

import rospy
import time
import sys

from lunabot_msgs.msg import RobotEffort, RobotSensors
from geometry_msgs.msg import Twist

from lunabot_control.pid_controller import VelocityPIDController
from lunabot_control.clamp_output import clamp_output
from lunabot_behavior.linear_actuators import LinearActuatorManager

from std_msgs.msg import Int8, Int32


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
        deposition_publisher: rospy.Publisher = None,
    ):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if excavation_publisher is None or lin_act_publisher is None or cmd_vel_publisher is None:
            self.excavation_publisher: rospy.Publisher = rospy.Publisher("/excavate", Int32, queue_size=1, latch=True)
            self.lin_act_publisher: rospy.Publisher = rospy.Publisher("/lin_act", Int32, queue_size=1, latch=True)
            self.cmd_vel_publisher: rospy.Publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1, latch=True)
            # testing trenching
            self.deposition_publisher = rospy.Publisher("/deposition", Int32, queue_size=1, latch=True)
            rospy.init_node("excavation_node")
        else:
            self.excavation_publisher: rospy.Publisher = excavation_publisher
            self.lin_act_publisher: rospy.Publisher = lin_act_publisher
            self.cmd_vel_publisher: rospy.Publisher = cmd_vel_publisher
            self.deposition_publisher: rospy.Publisher = deposition_publisher

        self.robot_sensors = RobotSensors()

        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.rate = rospy.Rate(10)

        # Constants (in meters)  TODO: update these 
        self.TARGET_DEPTH_OF_CUT = 0.005  # (.5 cm)
        self.BUCKET_RADIUS = 0.0948
        self.BUCKET_SPACING = 0.0853

        self.LOWERING_TIME = 30  # seconds
        self.TRENCHING_TIME = 30  # seconds

        self.LOAD_CELL_WEIGHT_THRESHOLD = 1  # In kg, TODO find value
        self.MAX_LIN_ACT_VEL = 0.00688405797  # In meters/s, the speed of the linear actuators at the max power (from experiment - 19 cm / 27.6 seconds)
        self.LIN_ACT_MAX_POWER = 110

        # TODO: tune further, 1500 is a bit slow for trenching, but higher values kick up a ton of dust
        self.EXCAVATION_SPEED = 1500
        
        # speed to run deposition during trenching (rpm)
        self.DEPOSITION_SPEED = 200
        
        # speed to run drivetrain during trenching (m/s)
        self.TRENCHING_SPEED = 0.01 # lil slow - exc stalled at 0.02, try 0.015 next

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

        excavation_message = Int32()
        lin_act_message = Int32()

        start_time = rospy.get_time()

        # Until the linear actuators reach the end (based on time), keep moving them down or the actuators reach the limit and stop
        while rospy.get_time() - start_time < self.LOWERING_TIME and self.robot_sensors.act_right_curr < self.LIN_ACT_CURR_THRESHOLD:

            excavation_message.data = self.EXCAVATION_SPEED

            excavation_velocity = self.robot_sensors.exc_vel

            # Set the target linear actuator velocity to target the depth of cut
            target_actuator_velocity = (
                self.TARGET_DEPTH_OF_CUT
                * excavation_velocity
                * self.BUCKET_RADIUS
                / self.BUCKET_SPACING
            )

            # No encoders on actuators so cannot do PID. Find the power based on the ratio of goal velocity to max velocity. Negative = go down
            lin_act_message.data = -1 * clamp_output(target_actuator_velocity / self.MAX_LIN_ACT_VEL * self.LIN_ACT_MAX_POWER)

            self.lin_act_publisher.publish(lin_act_message)
            self.excavation_publisher.publish(excavation_message)

            # Check for excavation getting stuck (high current)
            if self.robot_sensors.exc_curr > self.EXCAVATION_CURR_THRESHOLD:
                self.exc_failure_counter += 1
                self.spin_excavation_backwards()

            if (self.exc_failure_counter >= 7):
                break

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

        excavation_message = Int32()
        lin_act_message = Int32()

        start_time = rospy.get_time()

        # Until the linear actuators reach the end (based on time), keep moving them down
        while rospy.get_time() - start_time < self.LOWERING_TIME:

            lin_act_message.data = -1 * self.LIN_ACT_MAX_POWER
            excavation_message.data = self.EXCAVATION_SPEED

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

        excavation_message = Int32()
        cmd_vel_message = Twist()

        start_time = rospy.get_time()

        #load_cell_weight = self.robot_sensors.load_cell_weights[0] + self.robot_sensors.load_cell_weights[1]
        load_cell_weight = 0
        # maybe TODO add logic for stopping if obstacles exist (both rocks and craters)

        # Until the set amount of time and while the robot is not yet full, keep moving the robot forward and spinning excavation
        while rospy.get_time() - start_time < self.TRENCHING_TIME and load_cell_weight < self.LOAD_CELL_WEIGHT_THRESHOLD:

            excavation_velocity = self.robot_sensors.exc_vel

            excavation_message.data = self.EXCAVATION_SPEED

            self.excavation_publisher.publish(excavation_message)

            # testing trench - remove later
            deposition_msg = Int32()
            deposition_msg.data = self.DEPOSITION_SPEED
            self.deposition_publisher.publish(deposition_msg)

            target_linear_vel = (
                self.TARGET_DEPTH_OF_CUT
                * excavation_velocity
                * self.BUCKET_RADIUS
                / self.BUCKET_SPACING
            ) #TODO: fix? it is very wrong

            # cmd_vel_message.linear.x = target_linear_vel * 10
            cmd_vel_message.linear.x = self.TRENCHING_SPEED 
            cmd_vel_message.angular.z = 0

            self.cmd_vel_publisher.publish(cmd_vel_message)

            # Check for excavation getting stuck (high current)
            if self.robot_sensors.exc_curr > self.EXCAVATION_CURR_THRESHOLD:
                self.exc_failure_counter += 1
                cmd_vel_message.linear.x *= 0.5
                
            # Check if excavation motor stopped due to stalling
            if self.robot_sensors.exc_curr < 0.01:
                #TODO: clear stall
                self.exc_failure_counter += 1
                self.spin_excavation_backwards()
                

            if (self.exc_failure_counter >= 7):
                break

            # load_cell_weight = (
            #    self.robot_sensors.load_cell_weights[0]
            #    + self.robot_sensors.load_cell_weights[1]
            # )

            load_cell_weight = 0

            self.rate.sleep()

        excavation_message.data = 0
        cmd_vel_message.linear.x = 0
        cmd_vel_message.angular.z = 0
        deposition_msg.data = 0

        self.excavation_publisher.publish(excavation_message)
        self.deposition_publisher.publish(deposition_msg)
        
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


        excavation_message = Int32()
        excavation_message.data = 0
        self.excavation_publisher.publish(excavation_message)
        rospy.sleep(1)

        excavation_message.data = self.EXCAVATION_SPEED * -1

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

    linear_actuators = LinearActuatorManager(excavate_module.lin_act_publisher)
    print("Exc: Raising")
    linear_actuators.raise_linear_actuators()
