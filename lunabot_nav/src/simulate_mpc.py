import rospy
from geometry_msgs.msg import PoseStamped


if __name__ = "main":
    odom_topic = rospy.get_param('/odom_topic')
    goal_topic = rospy.get_param('/nav_goal_topic')

    goal = PoseStamped()
    goal.pose.position.x = 1
    goal.pose.position.y = 1
    goal.pose.position.z = 0

    goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
    