#!/usr/bin/env python3

import rosgraph
import rospy
import rostopic
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

if __name__ == "__main__":
    rospy.init_node("test_mpc")
    path_pub = rospy.Publisher("/lunabot_nav/path_generator", Path, queue_size=10)
    goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=1)

    master = rosgraph.Master("/rostopic")

    while True:
        pubs, subs = rostopic.get_topic_list(master=master)
        for topic in subs:
            should_break = False
            if topic[0] == "/goal":
                should_break = True
                break
        if should_break:
            break

    pose = PoseStamped()

    pose.pose.position.x = 2
    pose.pose.position.y = 0
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    goal_pub.publish(pose)

    points = [[0, 0], [2, 0]]

    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()
    for point in points:
        temp_pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.header.frame_id = "base_link"
        pose.header.stamp = rospy.Time.now()
        path.poses.append(temp_pose)

    path_pub.publish(path)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        path_pub.publish(path)
        goal_pub.publish(pose)
        rate.sleep()
