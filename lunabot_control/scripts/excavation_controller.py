#!/usr/bin/env python3
import numpy as np
import rospy

from lunabot_msgs.msg import RobotEffort, RobotState


class LeakyBucket:
    def __init__(self, condition_fn, inc=100, dec=10, capacity=1000):
        self.condition_fn = condition_fn
        self.inc_ = inc
        self.dec_ = dec
        self.capacity_ = capacity
        self.overflow_ = False
        self.count_ = 0

    def is_overflow(self):
        if self.condition_fn():
            self.count_ += self.inc_
        else:
            self.count_ = max(0, self.count_ - self.dec_)
        if self.count_ >= self.capacity_:
            self.overflow_ = True
        else:
            self.overflow_ = False
        return self.overflow_


class ExcavationController:
    max_exc_percent = 0.3
    max_lead_screw_percent = 0.5

    FORWARD_EXCAVATE = -1
    DOWN_LEAD_SCREW = -1

    valid_exc_range = (11000, 17400)
    # valid_exc_range = (5000,17400)
    # valid_lead_screw_range = (9000, 17000)
    valid_lead_screw_range = (6000, 22000)

    def __init__(self):
        self.exc_curr = None
        self.lead_screw_curr = None

        self.exc_voltage = self.FORWARD_EXCAVATE
        self.lead_screw_voltage = self.DOWN_LEAD_SCREW

        self.exc_leaky_bucket = LeakyBucket(self.is_exc_stuck, dec=20)
        self.lead_screw_leaky_bucket = LeakyBucket(self.is_lead_screw_invalid, dec=0)

        self._effort_pub = rospy.Publisher("/effort", RobotEffort, queue_size=1)
        self._state_sub = rospy.Subscriber("/state", RobotState, self._robot_state_cb)

        rospy.on_shutdown(self.shutdown_hook)

        self.rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            if self.exc_curr is not None and self.lead_screw_curr is not None:
                self.loop()
            self.rate.sleep()

    def _robot_state_cb(self, msg):
        self.exc_curr = msg.exc_curr
        self.lead_screw_curr = msg.lead_screw_curr

    def is_exc_stuck(self):
        return (
            self.exc_curr < self.valid_exc_range[0]
            or self.exc_curr > self.valid_exc_range[1]
        )

    def is_lead_screw_invalid(self):
        return (
            self.lead_screw_curr < self.valid_lead_screw_range[0]
            or self.lead_screw_curr > self.valid_lead_screw_range[1]
        )

    def loop(self):
        exc_overflow = self.exc_leaky_bucket.is_overflow()
        lead_screw_overflow = self.lead_screw_leaky_bucket.is_overflow()
        if exc_overflow:
            print("EXC OVERFLOW")
            self.lead_screw_voltage = -self.DOWN_LEAD_SCREW
        else:
            self.lead_screw_voltage = self.DOWN_LEAD_SCREW

        if lead_screw_overflow:
            print("LEAD_SCREW_OVERFLOW")
            self.lead_screw_voltage = 0

        effort_msg = RobotEffort()
        effort_msg.lead_screw = self.constrain(
            self.lead_screw_voltage, max_percent=self.max_lead_screw_percent
        )
        effort_msg.excavate = self.constrain(
            self.exc_voltage, max_percent=self.max_exc_percent
        )
        self._effort_pub.publish(effort_msg)

    def constrain(self, val, max_percent):
        val = np.clip(-1, val, 1)  # Clipping speed to not go over 100%
        return np.int8(val * 127 * max_percent)

    def stop(self):
        effort_msg = RobotEffort()
        self._effort_pub.publish(effort_msg)

    def shutdown_hook(self):
        self.stop()
        rospy.logerr("stopping excavation control")


if __name__ == "__main__":
    rospy.init_node("excavation_controller")
    controller = ExcavationController()
