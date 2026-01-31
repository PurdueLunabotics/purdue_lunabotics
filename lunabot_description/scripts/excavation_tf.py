#!/usr/bin/env python3

import numpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import rclpy

from tf2_ros import Buffer, Time, TransformBroadcaster, TransformListener, TransformStamped
import tf_transformations

class ExcavationTf(Node):
    def __init__(self, **kwargs):
        super().__init__("excation_tf_node", **kwargs)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.initial_tf = None
        
        self.setup_timer = self.create_timer(0.2, self.check_tf)
    
    def check_tf(self):
        try:
            self.initial_tf = self.tf_buffer.lookup_transform("middle_link", "excavation", Time())
            self.setup_timer.cancel()
            self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        except:
            self.get_logger().warn("Could not find link")

    def on_joint_states(self, states: JointState):
        if (self.initial_tf == None):
            self.get_logger().warn("Missing initial transform")
            return

        idx = states.name.index("excavation_joint")
        pos = states.position[idx]

        quaternion = self.initial_tf.transform.rotation
        rot = tf_transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        actuator_excavation = [pos, 0, 0, 0]
        translation = numpy.matmul(rot, actuator_excavation)
        new_tf = TransformStamped()

        new_tf.header.frame_id = self.initial_tf.header.frame_id
        new_tf.header.frame_id = self.initial_tf.header.frame_id
        new_tf.header.stamp = self.get_clock().now().to_msg()

        new_tf.child_frame_id = self.initial_tf.child_frame_id

        new_tf.transform.rotation.x = new_tf.transform.rotation.x
        new_tf.transform.rotation.y = new_tf.transform.rotation.y
        new_tf.transform.rotation.z = new_tf.transform.rotation.z
        new_tf.transform.rotation.w = new_tf.transform.rotation.w

        new_tf.transform.translation.x = new_tf.transform.translation.x + translation[0]
        new_tf.transform.translation.y = new_tf.transform.translation.y + translation[1]
        new_tf.transform.translation.z = new_tf.transform.translation.z + translation[2]

        self.tf_broadcaster.sendTransform(new_tf)


def main():
    rclpy.init()
    node = ExcavationTf()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
