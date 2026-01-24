#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformBroadcaster, TransformStamped

class SimOdom(Node):
    def __init__(self):
        super().__init__("sim_odom_node")

        self.ns = self.get_namespace().lstrip('/')

        self.odom_listener = self.create_subscription(PoseStamped, "gazebo/odom", self.__odom_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "rtabmap/odom", 1)
        self.broadcaster = TransformBroadcaster(self)

    def __odom_callback(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = f"{self.ns}odom"
        t.child_frame_id = f"{self.ns}base_link"

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation.w = msg.pose.orientation.w
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z

        self.broadcaster.sendTransform(t)

        odom = Odometry()
        
        odom.pose.pose = msg.pose
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = f"{self.ns}odom"

        self.odom_publisher.publish(odom)

def main():
    rclpy.init()
    sim_odom_node = SimOdom()

    rclpy.spin(sim_odom_node)

    sim_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
