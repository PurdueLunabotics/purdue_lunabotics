#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Transform, Quaternion

from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from tf2_ros.buffer import Buffer

class MapBase(Node):
    def __init__(self):
        super().__init__("map_base_node")

        ns = self.get_namespace().lstrip('/')
        if len(ns) != 0:
            ns = ns + '/';

        # transform we're looking for is from base link back to map
        self.from_frame_rel = f"{ns}base_link"
        self.to_frame_rel = f"{ns}map"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_listener = self.create_subscription(Odometry, "odom", self.__odom_callback, 10)
        self.position_publisher = self.create_publisher(PoseStamped, "position", 10)

    def __odom_callback(self, msg: Odometry):
        try:
            t = self.tf_buffer.lookup_transform(self.to_frame_rel, self.from_frame_rel, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"Could not transform {self.from_frame_rel} to {self.to_frame_rel}: {e}")
            return
        
        # create new pose stamped object with the transform data
        map_pose = PoseStamped()

        map_pose = tf2_geometry_msgs.do_transform_pose_stamped(map_pose, t)

        map_pose.header = Header()
        map_pose.header.stamp = msg.header.stamp
        map_pose.header.frame_id = self.to_frame_rel # transformed pose should be in "map"

        self.position_publisher.publish(map_pose)

def main():
    rclpy.init()
    map_pose_node = MapBase()

    rclpy.spin(map_pose_node)

    map_pose_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
