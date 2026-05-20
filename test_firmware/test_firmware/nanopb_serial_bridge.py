import rclpy
from . import RobotMsgs_pb2
import serial
from rclpy.node import Node

from lunabot_msgs.msg import RobotEffort, RobotSensors


class NanopbSerialBridge(Node):
    def __init__(self):
        super().__init__("nanopb_serial_bridge")

        # setup serial port -> match teensy port and baud rate
        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)

        self.sub = self.create_subscription(
            RobotEffort, "/robot_effort", self.effort_callback, 10
        )
        self.pub = self.create_publisher(RobotSensors, "/robot_state", 10)

        self.timer = self.create_timer(0.01, self.read_from_serial)

    def effort_callback(self, msg):
        # turns the ros2 msg into protobuf format
        effort_pb = RobotMsgs_pb2.RobotEffort()
        effort_pb.lin_act = msg.lin_act
        effort_pb.left_drive = msg.left_drive
        effort_pb.right_drive = msg.right_drive
        effort_pb.excavate = msg.excavate
        effort_pb.deposit = msg.deposit
        effort_pb.should_reset = msg.should_reset
        self.ser.write(
            effort_pb.SerializeToString() + b"\x00"
        )  # apparently returns some bytes obj

    def read_from_serial(self):
        if self.ser.in_waiting > 0:  # number of bytes in buf
            raw_data = self.ser.read_until(b"\x00")  # read buf
            # TODO: Deserialize raw_data using Nanopb and publish to /robot_state
            if not raw_data.endswith(b"\x00"):
                print("incomplete packet detected")
                return

            payload = raw_data[:-1]  # ignore delimiter

            try:
                state_pb = RobotMsgs_pb2.RobotSensors()
                state_pb.ParseFromString(payload)

                ros_msg = RobotSensors()
                ros_msg.act_right_curr = state_pb.act_right_curr
                ros_msg.dep_curr = state_pb.dep_curr
                ros_msg.exc_curr = state_pb.exc_curr
                ros_msg.drive_left_curr = state_pb.drive_left_curr
                ros_msg.drive_right_curr = state_pb.drive_right_curr
                ros_msg.exc_torque = state_pb.exc_torque
                ros_msg.drive_left_vel = state_pb.drive_left_vel
                ros_msg.drive_right_vel = state_pb.drive_right_vel
                ros_msg.exc_vel = state_pb.exc_vel
                ros_msg.act_left_pos = state_pb.act_left_pos
                ros_msg.act_right_pos = state_pb.act_right_pos
                self.pub.publish(ros_msg)
            except Exception as e:
                self.get_logger().error(f"Protobuf Error: {e}")
            pass


def main(args=None):
    rclpy.init(args=args)
    node = NanopbSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
