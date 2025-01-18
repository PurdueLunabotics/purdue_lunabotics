import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import math

class Zone:
    def __init__(self, top_left: 'tuple[float]', top_right: 'tuple[float]', bottom_left: 'tuple[float]', bottom_right: 'tuple[float]'):
        """
        Initialize a zone with four corners. Also keeps track of the middle of the zone
        """

        # tuples are (x, y) where x is left-right and y is bottom-top
        self.top_left = top_left
        self.top_right = top_right
        self.bottom_left = bottom_left
        self.bottom_right = bottom_right

        self.middle = ((bottom_left[0] + top_right[0]) / 2, (bottom_left[1] + top_right[1]) / 2)

    def visualize_zone(self, publisher: rospy.Publisher):
        """
        Visualizes a given zone (its four corners) as a square in rviz.
        Publisher should be a rospy publisher that publishes the Path message type. 
        (Make sure the publisher's topic is being visualized in rviz)
        """

        # Make a path containing all of the corners of the zone. Make it into a path, and use the self.publisher to publish it.
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "odom"

        path.poses.append(PoseStamped())
        path.poses[-1].header.frame_id = "odom"

        path.poses[-1].pose.position.x = self.top_left[0]
        path.poses[-1].pose.position.y = self.top_left[1]

        path.poses.append(PoseStamped())
        path.poses[-1].header.frame_id = "odom"

        path.poses[-1].pose.position.x = self.top_right[0]
        path.poses[-1].pose.position.y = self.top_right[1]

        path.poses.append(PoseStamped())
        path.poses[-1].header.frame_id = "odom"

        path.poses[-1].pose.position.x = self.bottom_right[0]
        path.poses[-1].pose.position.y = self.bottom_right[1]

        path.poses.append(PoseStamped())
        path.poses[-1].header.frame_id = "odom"

        path.poses[-1].pose.position.x = self.bottom_left[0]
        path.poses[-1].pose.position.y = self.bottom_left[1]

        path.poses.append(PoseStamped())
        path.poses[-1].header.frame_id = "odom"

        path.poses[-1].pose.position.x = self.top_left[0]
        path.poses[-1].pose.position.y = self.top_left[1]

        publisher.publish(path)

def calc_point_from_apriltag(x: float, y: float, apriltag_pose_in_odom: PoseStamped, is_sim: bool)-> 'tuple[float,float]':
    """
    Calculates the (x, y) point from the apriltag's location (given in the odom frame)
    """

    roll, pitch, yaw  = euler_from_quaternion([apriltag_pose_in_odom.pose.orientation.x, apriltag_pose_in_odom.pose.orientation.y, apriltag_pose_in_odom.pose.orientation.z, apriltag_pose_in_odom.pose.orientation.w])

    # Add pi/2 in sim to the initial angle because the apriltag orientation in the odom frame is 90 degrees off (it points to the right instead of outwards)
    # In the real robot, subtract (go the other way)

    if (is_sim):
        yaw += math.pi / 2
    else:
        yaw -= math.pi / 2 

    x0 = apriltag_pose_in_odom.pose.position.x
    y0 = apriltag_pose_in_odom.pose.position.y

    x1 = x0 + math.cos(yaw) * x - math.sin(yaw) * y
    y1 = y0 + math.sin(yaw) * x + math.cos(yaw) * y

    return (x1, y1)

def calc_offset(x: float, y: float, apriltag_pose_in_odom: PoseStamped, is_sim: bool)-> 'tuple[float,float]':
    """
    Calc the offset from the apriltag's location (without inbuilt position)
    """

    roll, pitch, yaw  = euler_from_quaternion([apriltag_pose_in_odom.pose.orientation.x, apriltag_pose_in_odom.pose.orientation.y, apriltag_pose_in_odom.pose.orientation.z, apriltag_pose_in_odom.pose.orientation.w])

    # Add pi/2 in sim to the initial angle because the apriltag orientation in the odom frame is 90 degrees off (it points to the right instead of outwards)
    # In the real robot, subtract (go the other way)

    if (is_sim):
        yaw += math.pi / 2
    else:
        yaw -= math.pi / 2 

    x0 = 0
    y0 = 0

    x1 = x0 + math.cos(yaw) * x - math.sin(yaw) * y
    y1 = y0 + math.sin(yaw) * x + math.cos(yaw) * y

    return (x1, y1)


def find_mining_zone(apriltag_pose_in_odom: PoseStamped, is_sim: bool)->Zone:

    DIST_X = 3.78 # In meters, the distance from the leftmost wall to the left border of the mining zone
    # KSC = 3.88
    LENGTH_X = 3 # In meters, the length of the mining zone (left to right)
    # KSC = 3

    DIST_Y = 1 # In meters, the distance from the bottom-most wall to the bottom border of the mining zone
    # KSC = 1
    LENGTH_Y = 3 # In meters, the length of the mining zone (bottom to top)
    # KSC = 3

    top_left = calc_point_from_apriltag(DIST_X, DIST_Y + LENGTH_Y, apriltag_pose_in_odom, is_sim)
    top_right = calc_point_from_apriltag(DIST_X + LENGTH_X, DIST_Y + LENGTH_Y, apriltag_pose_in_odom, is_sim)
    bottom_left = calc_point_from_apriltag(DIST_X, DIST_Y, apriltag_pose_in_odom, is_sim)
    bottom_right = calc_point_from_apriltag(DIST_X + LENGTH_X, DIST_Y, apriltag_pose_in_odom, is_sim)

    return Zone(top_left, top_right, bottom_left, bottom_right)

def find_berm_zone(apriltag_pose_in_odom: PoseStamped, is_sim: bool)->Zone:

    DIST_X = 4.78
    # KSC = ~4.88
    LENGTH_X = 2
    # KSC = ~2

    DIST_Y = 0
    # KSC = ~0.1
    LENGTH_Y = -1
    # KSC = ~1

    top_left = calc_point_from_apriltag(DIST_X, DIST_Y + LENGTH_Y, apriltag_pose_in_odom, is_sim)
    top_right = calc_point_from_apriltag(DIST_X + LENGTH_X, DIST_Y + LENGTH_Y, apriltag_pose_in_odom, is_sim)
    bottom_left = calc_point_from_apriltag(DIST_X, DIST_Y, apriltag_pose_in_odom, is_sim)
    bottom_right = calc_point_from_apriltag(DIST_X + LENGTH_X, DIST_Y, apriltag_pose_in_odom, is_sim)

    return Zone(top_left, top_right, bottom_left, bottom_right)
