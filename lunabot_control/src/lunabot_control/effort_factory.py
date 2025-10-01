#!/usr/bin/python3

from rclpy.node import Node
import threading
import rclpy

from lunabot_msgs.msg import RobotEffort, RobotErrors
from std_msgs.msg import Int8, Int32, Bool


class EffortFactory(Node):
    """
    Compiles all of the effort values into a single message and publishes it. Avoids issues with multiple publishers all resetting the effort message.
    """

    def __init__(self, **kwargs):
        super().__init__('effort_factory_node', **kwargs)
        rclpy.get_global_executor().add_node(self)

        self.effort = RobotEffort()
        self.lin_act = 0
        self.left_drive = 0
        self.right_drive = 0
        self.excavate = 0
        self.deposition = 0
        self.should_reset = False

        self.declare_parameter("autonomy", True)

        self.robot_errors = RobotErrors()

        self.effort_publisher = self.create_publisher(
            RobotEffort, "/effort",  10
        )

        self.lin_act_subscriber = self.create_subscription(Int32, "/lin_act", self.set_lin_act, 1)
        self.left_drive_subscriber = self.create_subscription(Int32, "/left_drive", self.set_left_drive, 1)
        self.right_drive_subscriber = self.create_subscription(Int32, "/right_drive", self.set_right_drive, 1)
        self.excavate_subscriber = self.create_subscription(Int32, "/excavate", self.set_excavate, 1)
        self.deposition_subscriber = self.create_subscription(Int32, "/deposition", self.set_deposition, 1)

        self.error_subscriber = self.create_subscription(RobotErrors, "/errors", self.error_callback, 1)
        self.stall_subscriber = self.create_subscription(Bool, "/stalled", self.stall_callback, 1)
        self.stall_publisher = self.create_publisher(Bool, "/stalled", 10)
        self.stalled = False
        self.cleared_stall = False

        rate = self.create_rate(50.0, self.get_clock())

        while rclpy.ok():
            if (self.get_parameter("autonomy").get_parameter_value().bool_value):
                if self.stalled == True: # if stalled, don't publish effort
                    self.fix_stall()
                elif self.robot_errors.manual_stop == False:
                    self.publish_effort()
                else:
                    self.stop()
            rate.sleep()

    def set_lin_act(self, lin_act: Int32):
        self.lin_act = lin_act.data

    def set_left_drive(self, left_drive: Int32):
        self.left_drive = left_drive.data

    def set_right_drive(self, right_drive: Int32):
        self.right_drive = right_drive.data

    def set_excavate(self, excavate: Int32):
        self.excavate = excavate.data

    def set_deposition(self, deposition: Int32):
        self.deposition = deposition.data

    def error_callback(self, msg: RobotErrors):
        self.robot_errors = msg

    def stall_callback(self, msg: Bool):
        self.stalled = msg.data
        self.cleared_stall = False

    def publish_effort(self):
        # print("published effort")
        self.effort.lin_act = self.lin_act
        self.effort.left_drive = self.left_drive
        self.effort.right_drive = self.right_drive
        self.effort.excavate = self.excavate
        self.effort.deposit = self.deposition
        self.effort.should_reset = self.should_reset

        self.effort_publisher.publish(self.effort)

    def stop(self):
        #print("stopped")
        self.effort.lin_act = 0
        self.effort.left_drive = 0
        self.effort.right_drive = 0
        self.effort.excavate = 0
        self.effort.deposit = 0

        self.effort_publisher.publish(self.effort)
    
    def fix_stall(self):
        self.get_logger().error("Fixing Stall")

        self.effort.lin_act = 0
        self.effort.left_drive = 0
        self.effort.right_drive = 0
        self.effort.excavate = 0
        self.effort.deposit = 0
        self.effort.should_reset = True
        
        self.effort_publisher.publish(self.effort)

        self.create_timer(0.5).sleep()

        # self.effort.lin_act = 100
        # self.effort.left_drive = 0
        # self.effort.right_drive = 0
        # self.effort.excavate = 0
        # self.effort.deposit = 0
        # self.effort.should_reset = False
        self.stalled = False
        self.should_reset = False
        
        # self.effort_publisher.publish(self.effort)
        self.stall_publisher.publish(False)
        
        # self.cleared_stall = True


def spin_in_background():
    executor = rclpy.get_global_executor()
    try:
        executor.spin()
    except ExternalShutdownException: # Not defined
        pass

def main():
    rclpy.init()
    t = threading.Thread(target=spin_in_background)
    t.start()
    controller = EffortFactory()
    controller.run_node()
