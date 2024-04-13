import rospy

from lunabot_msgs.msg import RobotEffort, RobotSensors
import interrupts

import time

class Plunge:
    '''
    This is a transition state that spins excavation and lowers the linear actuators to begin mining.
    '''
    
    def sensors_callback(self, msg: RobotSensors):
        self.robot_sensors = msg

    def __init__(self, effort_publisher: rospy.Publisher = None):
        """
        If passed a publisher, then it is assumed a node is already running, and the publisher is shared.
        Else, initialize this node to run on its own.
        """

        if effort_publisher is None:
            self.effort_publisher = rospy.Publisher("/effort", RobotEffort, queue_size=1, latch=True)
            rospy.init_node('plunge_node')
        else:
            self.effort_publisher = effort_publisher

        self.robot_sensors = RobotSensors()

        rospy.Subscriber("/sensors", RobotSensors, self.sensors_callback)

        self.rate = rospy.Rate(10)

        # TODO: check and test this value
        self.LOWERING_TIME = 5 #In seconds, how long it takes to lower the linear actuators 90% of the way

        self.is_sim = rospy.get_param("is_sim")

    def plunge(self):
        """
        Spin excavation at full speed, lower linear actuators 90% to begin mining, and then reduce speed to 25%
        """

        if (self.is_sim):
            rospy.loginfo("Plunge: would plunge")
            time.sleep(2)
            return True

        time.sleep(0.1)

        effort_message = RobotEffort()
        effort_message.excavate = True

        effort_message.lin_act = -127 #TODO check if the direction is right/what the max is

        start_time = rospy.get_time()

        while (rospy.get_time() - start_time < self.LOWERING_TIME):
            self.effort_publisher.publish(effort_message)

            if (interrupts.check_for_interrupts() != interrupts.Errors.FINE):
                return False

            self.rate.sleep()

        effort_message.lin_act = 0
        effort_message.excavate = int(0.25 * 127) #25% speed

        self.effort_publisher.publish(effort_message)

        return True
    
if __name__ == "__main__":
    plunge = Plunge()
    plunge.plunge()
