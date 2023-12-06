import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler

DISTANCE = 6  # meters

ALPHA = 0.75  # what percent is straight?

STEP = 0.1  # meters


if __name__ == "__main__":
    args = sys.argv

    if len(args) != 3:
        print("Usage: python fake_paths.py <degrees> <left/right>")
        exit()

    try:
        degrees = int(args[1])
    except ValueError:
        print("Usage: python fake_paths.py <degrees> <left/right>")
        exit()

    if not (0 <= degrees < 180):
        print("Usage: python fake_paths.py <degrees> <left/right>")
        exit()

    if args[2] != "left" and args[2] != "right":
        print("Usage: python fake_paths.py <degrees> <left/right>")
        exit()

    rospy.init_node("fake_path_node")

    rate = rospy.Rate(10)  # hz

    angle = None  # Rad
    position = None  # meters, [x,y]

    def position_subscriber(data: Odometry):
        global angle
        global position
        mypose: Odometry = data

        quaternion = mypose.pose.pose.orientation
        eulerAngles = euler_from_quaternion(
            [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
        )
        yaw = eulerAngles[2]

        angle = yaw

        position = [mypose.pose.pose.position.x, mypose.pose.pose.position.y]

    odom_topic = rospy.get_param("/odom_topic")
    rospy.Subscriber(odom_topic, Odometry, position_subscriber)

    path_publisher = rospy.Publisher("/nav/global_path", Path, queue_size=10)

    path = []

    while position is None:
        rate.sleep()

    # Append the straight part
    i = 0
    while i < DISTANCE * ALPHA:
        position = [
            position[0] + STEP * math.cos(angle),
            position[1] + STEP * math.sin(angle),
        ]
        path.append((position[0], position[1]))
        i += STEP

    currentAngle = math.degrees(angle)

    if args[2] == "left":
        newAngle = (currentAngle + degrees) % (360)
        angle = math.radians(newAngle)

    if args[2] == "right":
        newAngle = (currentAngle - degrees) % (360)
        angle = math.radians(newAngle)

    # Append the turn part
    while i <= DISTANCE:
        position = [
            position[0] + STEP * math.cos(angle),
            position[1] + STEP * math.sin(angle),
        ]
        path.append((position[0], position[1]))
        i += STEP

    ros_struct = Path()

    ros_struct.poses = []
    ros_struct.header.stamp = rospy.Time.now()
    ros_struct.header.frame_id = "odom"

    for point in path:
        path_pose = PoseStamped()

        path_pose.pose.position.x = point[0]
        path_pose.pose.position.y = point[1]

        rotation = quaternion_from_euler(0, 0, 0)
        path_pose.pose.orientation.x = rotation[0]
        path_pose.pose.orientation.y = rotation[1]
        path_pose.pose.orientation.z = rotation[2]
        path_pose.pose.orientation.w = rotation[3]

        ros_struct.poses.append(path_pose)

    print("published")

    while 1:
        path_publisher.publish(ros_struct)
        for i in range(10):
            rate.sleep()
