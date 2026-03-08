#!/usr/bin/python3

from rclpy.node import Node
import rclpy
import math

from lunabot_msgs.msg import RobotEffort, RobotSensors
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class DrivetrainController(Node):
    def __init__(self, **kwargs):
        super().__init__('drivetrain_controller_node', **kwargs)

        self.declare_parameter("gearbox_ratio", 10.0)
        self.declare_parameter("do_excavation", True)
        self._gearbox_ratio = self.get_parameter("gearbox_ratio").get_parameter_value().double_value
        self.do_excavation = self.get_parameter("do_excavation").get_parameter_value().bool_value

        self.effort_subscriber = self.create_subscription(RobotEffort, "effort", self.effort_callback, 1)
        self.velocity_commands_publisher = self.create_publisher(Float64MultiArray, "velocity_controller/commands", 10)
        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_cb, 10)
        self.sensor_pub = self.create_publisher(RobotSensors, "sensors", 10)

        self.joint_state = None
        self.target_excavation_vel = 0

    def joint_state_cb(self, state: JointState):
        if not self.do_excavation:
            return

        sensors = RobotSensors()

        excavation_idx = state.name.index("excavation_joint")
        if abs(state.velocity[excavation_idx] - self.target_excavation_vel) > 0.1:
            sensors.act_right_curr = 0.0
        else:
            sensors.act_right_curr = 2.0
        self.sensor_pub.publish(sensors)

    def scale_rpm(self, rpm):
        return rpm / 60.0 * 2.0 * math.pi / self._gearbox_ratio

    def effort_callback(self, effort: RobotEffort):
        output = Float64MultiArray()
        
        left = self.scale_rpm(float(effort.left_drive))
        right = self.scale_rpm(float(effort.right_drive))

        if self.do_excavation:
            self.target_excavation_vel = -effort.lin_act / 512
            output.data = [left, left,
                           right, right, self.target_excavation_vel]
        else:
            output.data = [left, left,
                           right, right]

        self.velocity_commands_publisher.publish(output)


def main():
    rclpy.init()
    controller = DrivetrainController()
    rclpy.spin(controller)
