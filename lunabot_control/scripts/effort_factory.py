#!/usr/bin/python3

import rospy

from lunabot_msgs.msg import RobotEffort
from std_msgs.msg import Int8

class EffortFactory():
    """
    Compiles all of the effort values into a single message and publishes it. Avoids issues with multiple publishers all resetting the effort message.
    """

    def __init__(self):
        self.effort = RobotEffort()
        self.lin_act = 0
        self.left_drive = 0
        self.right_drive = 0
        self.excavate = 0
        self.deposition = 0
        
        self.effort_publisher = rospy.Publisher("/effort", RobotEffort, queue_size=5, latch=True)

        self.lin_act_subscriber = rospy.Subscriber("/lin_act", Int8, self.set_lin_act)
        self.left_drive_subscriber = rospy.Subscriber("/left_drive", Int8, self.set_left_drive)
        self.right_drive_subscriber = rospy.Subscriber("/right_drive", Int8, self.set_right_drive)
        self.excavate_subscriber = rospy.Subscriber("/excavate", Int8, self.set_excavate)
        self.deposition_subscriber = rospy.Subscriber("/deposition", Int8, self.set_deposition)

        self.rate = rospy.Rate(50)

    def set_lin_act(self, lin_act: int):
        self.lin_act = lin_act

    def set_left_drive(self, left_drive: int):
        self.left_drive = left_drive

    def set_right_drive(self, right_drive: int):
        self.right_drive = right_drive

    def set_excavate(self, excavate: int):
        self.excavate = excavate

    def set_deposition(self, deposition: int):
        self.deposition = deposition

    def publish_effort(self):
        self.effort.lin_act = self.lin_act
        self.effort.left_drive = self.left_drive
        self.effort.right_drive = self.right_drive
        self.effort.excavate = self.excavate
        self.effort.deposit = self.deposition

        self.effort_publisher.publish(self.effort)


if __name__ == "__main__":
    rospy.init_node("effort_publisher_node")

    effort_factory = EffortFactory()

    while not rospy.is_shutdown():
        effort_factory.publish_effort()
        effort_factory.rate.sleep()
