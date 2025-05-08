import rospy
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import math
import random

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
    
    def random_offset(self) -> 'tuple[float]':
        """
        Generate a random offset that will lie within the zone.
        """

        BUFFER = 0.1 # In meters, offset will not lie within this distance of edges

        x_length = self.top_right[0] - self.top_left[0]
        y_length = self.top_left[1] - self.bottom_left[1]

        x_offset = random.uniform(-x_length / 2 + BUFFER, x_length / 2 - BUFFER)
        y_offset = random.uniform(-y_length / 2 + BUFFER, y_length / 2 - BUFFER)

        return (x_offset, y_offset)

    def visualize_zone(self, publisher: rospy.Publisher, id=0, color=(1,0,0,1)):
        """
        Visualizes a given zone (its four corners) as a square in rviz.
        Publisher should be a rospy publisher that publishes the Marker message type. 
        (Make sure the publisher's topic is being visualized in rviz)
        id should be unique for each marker.
        Color is optional, given in (r,g,b,a) between 0 and 1.
        """

        # Make a polygon containing all of the corners of the zone and publish it
        zone = Marker()
        zone.header.stamp = rospy.Time.now()
        zone.header.frame_id = "odom"

        zone.ns = "zones"
        zone.id = id
        zone.type = Marker.LINE_STRIP
        zone.action = Marker.ADD

        zone.scale.x = 0.05
        zone.pose.position.x = 0
        zone.pose.position.y = 0
        zone.pose.position.z = 0

        zone.pose.orientation.x = 0
        zone.pose.orientation.y = 0
        zone.pose.orientation.z = 0
        zone.pose.orientation.w = 1

        zone.color.r = color[0]
        zone.color.g = color[1]
        zone.color.b = color[2]
        zone.color.a = color[3]
        
        zone.lifetime = rospy.Duration(0, 0)  #  duration of 0 = infinite


        p1 = Point()
        p1.x = self.top_left[0]
        p1.y = self.top_left[1]
        p1.z = 0

        p2 = Point()
        p2.x = self.top_right[0]
        p2.y = self.top_right[1]
        p2.z = 0

        p3 = Point()
        p3.x = self.bottom_right[0]
        p3.y = self.bottom_right[1]
        p3.z = 0

        p4 = Point()
        p4.x = self.bottom_left[0]
        p4.y = self.bottom_left[1]
        p4.z = 0

        p5 = Point()
        p5.x = self.top_left[0]
        p5.y = self.top_left[1]
        p5.z = 0

        zone.points = [p1, p2, p3, p4, p5]

        publisher.publish(zone)

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
        yaw += math.pi / 2 

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
        yaw += math.pi / 2 

    x0 = 0
    y0 = 0

    x1 = x0 + math.cos(yaw) * x - math.sin(yaw) * y
    y1 = y0 + math.sin(yaw) * x + math.cos(yaw) * y

    return (x1, y1)


def find_mining_zone(apriltag_pose_in_odom: PoseStamped, is_sim: bool)->Zone:
    """ 
    Calculate the mining zone from the apriltag's location.
    Assume the start apriltag is in the starting zone, 0.5 m +y and 0.1 m +x (0.1 m off of left wall, 0.5 m up)
    """

    DIST_X = 5.44 # In meters, the distance from the leftmost wall to the left border of the mining zone
    # KSC = 3.78
    # UCF 'top/bottom' = 5.44

    LENGTH_X = 2.6 # In meters, the length of the mining zone (left to right)
    # KSC = 3
    # UCF 'top/bottom' = 2.6

    DIST_Y = 0.5 # In meters, the distance from the bottom-most wall to the bottom border of the mining zone
    # KSC = 1.5
    # UCF 'top' = 0.5
    # UCF 'bottom' = -0.5

    LENGTH_Y = -2.57 # In meters, the length of the mining zone (bottom to top)
    # KSC = 3
    # UCF 'top' = -2.57
    # UCF 'bottom' = 2.57

    top_left = calc_point_from_apriltag(DIST_X, DIST_Y + LENGTH_Y, apriltag_pose_in_odom, is_sim)
    top_right = calc_point_from_apriltag(DIST_X + LENGTH_X, DIST_Y + LENGTH_Y, apriltag_pose_in_odom, is_sim)
    bottom_left = calc_point_from_apriltag(DIST_X, DIST_Y, apriltag_pose_in_odom, is_sim)
    bottom_right = calc_point_from_apriltag(DIST_X + LENGTH_X, DIST_Y, apriltag_pose_in_odom, is_sim)

    return Zone(top_left, top_right, bottom_left, bottom_right)

def find_berm_zone(apriltag_pose_in_odom: PoseStamped, is_sim: bool)->Zone:

    DIST_X = 5.94
    # KSC = ~ 4.45
    # UCF 'top/bottom' = ~ 5.94

    LENGTH_X = 1.5
    # KSC = ~2
    # UCF 'top/bottom' = ~ 1.5

    DIST_Y = -2.57
    # KSC = ~ 0.45
    # UCF 'top' = ~ -2.57
    # UCF 'bottom' = ~ 2.57

    LENGTH_Y = -0.9
    # KSC = ~ -0.7  (negative, because I'm defining the start as the top, and going down)
    # UCF 'top' = ~ -0.9
    # UCF 'bottom' = ~ 0.9

    top_left = calc_point_from_apriltag(DIST_X, DIST_Y + LENGTH_Y, apriltag_pose_in_odom, is_sim)
    top_right = calc_point_from_apriltag(DIST_X + LENGTH_X, DIST_Y + LENGTH_Y, apriltag_pose_in_odom, is_sim)
    bottom_left = calc_point_from_apriltag(DIST_X, DIST_Y, apriltag_pose_in_odom, is_sim)
    bottom_right = calc_point_from_apriltag(DIST_X + LENGTH_X, DIST_Y, apriltag_pose_in_odom, is_sim)

    return Zone(top_left, top_right, bottom_left, bottom_right)
