import rospy

from lunabot_msgs.msg import RobotSensors
from std_msgs.msg import Int8, Int32

import time

class LinearActuatorManager:
    '''
    Used to raise or lower linear actuators
    '''

    def sensors_callback(self, msg: RobotSensors):
        self.robot_sensors = msg

    def __init__(self, lin_act_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if lin_act_publisher is None:
            self.lin_act_publisher = rospy.Publisher("/lin_act", Int32, queue_size=1, latch=True)
            rospy.init_node('lin_act_node')
        else:
            self.lin_act_publisher = lin_act_publisher

        self.robot_sensors = RobotSensors()

        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.rate = rospy.Rate(10)  # 10hz

        self.ACTUATOR_CURRENT_THRESHOLD = 0.1 #TODO find best value or remove

        self.RAISING_TIME = 30
        self.WAITING_TIME = 5

        self.LIN_ACT_POWER = 110

        self.is_sim = rospy.get_param("is_sim")

    def raise_linear_actuators(self, use_current: bool = False):
        """
        Raise linear actuators to the max. height by turning them on until the current received is 0.
        """

        # don't run if in sim
        if (self.is_sim):
            rospy.loginfo("Behavior: would raise actuators")
            time.sleep(4)
            return True

        time.sleep(0.1)

        rospy.loginfo("Behavior: raising actuators")

        lin_act_msg = Int32()
        lin_act_msg.data = self.LIN_ACT_POWER

        start_time = rospy.get_time()

        # raise for the given time, at max
        while (rospy.get_time() - start_time < self.RAISING_TIME):
            self.lin_act_publisher.publish(lin_act_msg)

            # if we are using current data, then stop if the current is close to 0
            if (use_current and rospy.get_time() - start_time > self.WAITING_TIME and (self.robot_sensors.act_right_curr - 0) < self.ACTUATOR_CURRENT_THRESHOLD):
                lin_act_msg.data = 0
                self.lin_act_publisher.publish(lin_act_msg)
                return True

        self.rate.sleep()

        lin_act_msg.data = 0
        self.lin_act_publisher.publish(lin_act_msg)
        
        return True

if __name__ == "__main__":
    lin_actuators = LinearActuatorManager()
    lin_actuators.raise_linear_actuators()
