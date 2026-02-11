import rclpy

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from lunabot_msgs.msg import Zone

from rclpy.node import Node

import time

# zone geometries - based on guidebook orientations (all measurements in meters)
START_OFFSET_X = 2.44
START_OFFSET_Y = 1.5
START_LENGTH_X = 2
START_LENGTH_Y = 2

EXC_OFFSET_X = 2.19
EXC_OFFSET_Y = 0
EXC_LENGTH_X = 2.5
EXC_LENGTH_Y = 5

BERM_OFFSET_X = -1.94
BERM_OFFSET_Y = 1.9
BERM_LENGTH_X = 1.7
BERM_LENGTH_Y = 0.8

class ZonesNode(Node):
    def __init__(self):
        super().__init__("zones_node")

        self.declare_parameter("sim", True, ParameterDescriptor(type = ParameterType.PARAMETER_BOOL))

        self.exc_zone_marker_pub = self.create_publisher(Marker, "/exc_zone_marker", 10)
        self.berm_zone_marker_pub = self.create_publisher(Marker, "/berm_zone_marker", 10)
        self.start_zone_marker_pub = self.create_publisher(Marker, "/start_zone_marker", 10)

        self.exc_zone_pub = self.create_publisher(Zone, "/exc_zone", 10)
        self.berm_zone_pub = self.create_publisher(Zone, "/berm_zone", 10)
        self.start_zone_pub = self.create_publisher(Zone, "/start_zone", 10)
        
        # self.exc_zone_pub = self.create_publisher(lunabot_msgs.Zone, "/exc_zone", 10)

        # self.create_subscription(AprilTagDetectionArray, "/d455_back/detections", self.apriltag_callback, 1)

        # self.START_APRILTAG_ID = 11

        self.exc_zone = None
        self.berm_zone = None
        self.start_zone = None

        # self.is_sim = self.get_parameter("sim").get_parameter_value().bool_value
        self.start_time = time.perf_counter_ns()

        self.start_zone = self.make_zone(
            START_OFFSET_X,
            START_OFFSET_Y,
            START_LENGTH_X,
            START_LENGTH_Y)
        
        self.exc_zone = self.make_zone(
            EXC_OFFSET_X,
            EXC_OFFSET_Y,
            EXC_LENGTH_X,
            EXC_LENGTH_Y)
        
        self.berm_zone = self.make_zone(
            BERM_OFFSET_X,
            BERM_OFFSET_Y,
            BERM_LENGTH_X,
            BERM_LENGTH_Y)


        # transform we're looking for is from base link back to map
        # self.from_frame_rel = f"{ns}base_link"
        # self.to_frame_rel = f"{ns}map"

    def make_zone(self, offset_x, offset_y, length_x, length_y):
        z = Zone()

        z.v1 = Point()
        z.v1.x = offset_x + (length_x / 2)
        z.v1.y = offset_y + (length_y / 2)

        z.v2 = Point()
        z.v2.x = offset_x + (length_x / 2)
        z.v2.y = offset_y - (length_y / 2)

        z.v3 = Point()
        z.v3.x = offset_x - (length_x / 2)
        z.v3.y = offset_y - (length_y / 2)

        z.v4 = Point()
        z.v4.x = offset_x - (length_x / 2)
        z.v4.y = offset_y + (length_y / 2)

        return z

    def visualize_zone(self, zone: Zone, publisher: rclpy.publisher.Publisher, id=0, color=(1.0, 0.0, 0.0, 1.0)):
        """
        Visualizes a given zone (its four corners) as a square in rviz.
        Publisher should be a rospy publisher that publishes the Marker message type. 
        (Make sure the publisher's topic is being visualized in rviz)
        id should be unique for each marker.
        Color is optional, given in (r,g,b,a) between 0 and 1.
        """

        # Make a polygon containing all of the corners of the zone and publish it
        zone_marker = Marker()
        duration = time.perf_counter_ns() - self.start_time

        zone_marker.header.stamp = rclpy.time.Time(nanoseconds=duration).to_msg()
        zone_marker.header.frame_id = "map"

        zone_marker.ns = "zones"
        zone_marker.id = id
        zone_marker.type = Marker.LINE_STRIP
        zone_marker.action = Marker.ADD

        zone_marker.scale.x = 0.05
        zone_marker.pose.position.x = 0.0
        zone_marker.pose.position.y = 0.0
        zone_marker.pose.position.z = 0.0

        zone_marker.pose.orientation.x = 0.0
        zone_marker.pose.orientation.y = 0.0
        zone_marker.pose.orientation.z = 0.0
        zone_marker.pose.orientation.w = 1.0

        zone_marker.color.r = color[0]
        zone_marker.color.g = color[1]
        zone_marker.color.b = color[2]
        zone_marker.color.a = color[3]
        
        # zone.lifetime = rospy.Duration(0, 0)  #  duration of 0 = infinite


        p1 = zone.v1
        p1.z = 0.0

        p2 = zone.v2
        p2.z = 0.0

        p3 = zone.v3
        p3.z = 0.0

        p4 = zone.v4
        p4.z = 0.0

        p5 = zone.v1 # close the loop
        p5.z = 0.0

        zone_marker.points = [p1, p2, p3, p4, p5]

        publisher.publish(zone_marker)

    def mainloop(self):
        # rate = self.create_rate(30) # 30 hz
        while rclpy.ok():
            # zone = Zone((1.0, 2.0), (2.0, 2.0), (1.0, 1.0), (2.0, 1.0))

            if (self.exc_zone is not None):
                self.visualize_zone(self.exc_zone, self.exc_zone_marker_pub)
                self.exc_zone_pub.publish(self.exc_zone)

            if (self.berm_zone is not None):
                self.visualize_zone(self.berm_zone, self.berm_zone_marker_pub)
                self.berm_zone_pub.publish(self.berm_zone)

            if (self.start_zone is not None):
                self.visualize_zone(self.start_zone, self.start_zone_marker_pub)
                self.start_zone_pub.publish(self.start_zone)
            # print("published zone")

            # rate.sleep()

def main():
    rclpy.init()
    zones_node = ZonesNode()

    # print("starting")
    zones_node.mainloop()

    zones_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()