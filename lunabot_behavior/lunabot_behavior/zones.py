#!/usr/bin/env python3

import rclpy

from geometry_msgs.msg import Point
from lunabot_msgs.msg import Zone
from visualization_msgs.msg import Marker
import shapely.geometry as shp
import time

from rclpy.node import Node

import numpy as np

# zone geometries - based on guidebook orientations (all measurements in meters)
class ZoneMeasurements:
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

def make_zone(offset_x, offset_y, length_x, length_y):
    z = Zone()

    z.v1 = Point() # top right
    z.v1.x = offset_x + (length_x / 2)
    z.v1.y = offset_y + (length_y / 2)

    z.v2 = Point() # bottom right
    z.v2.x = offset_x + (length_x / 2)
    z.v2.y = offset_y - (length_y / 2)

    z.v3 = Point() # bottom left
    z.v3.x = offset_x - (length_x / 2)
    z.v3.y = offset_y - (length_y / 2)

    z.v4 = Point() # top left
    z.v4.x = offset_x - (length_x / 2)
    z.v4.y = offset_y + (length_y / 2)

    return z

start_zone = make_zone(
    ZoneMeasurements.START_OFFSET_X,
    ZoneMeasurements.START_OFFSET_Y,
    ZoneMeasurements.START_LENGTH_X,
    ZoneMeasurements.START_LENGTH_Y)

exc_zone = make_zone(
    ZoneMeasurements.EXC_OFFSET_X,
    ZoneMeasurements.EXC_OFFSET_Y,
    ZoneMeasurements.EXC_LENGTH_X,
    ZoneMeasurements.EXC_LENGTH_Y)

berm_zone = make_zone(
    ZoneMeasurements.BERM_OFFSET_X,
    ZoneMeasurements.BERM_OFFSET_Y,
    ZoneMeasurements.BERM_LENGTH_X,
    ZoneMeasurements.BERM_LENGTH_Y)

# TODO: replace with actual point to line calculation
def get_distance_from_start(p: np.array):
    start_center = np.array([ZoneMeasurements.START_OFFSET_X, ZoneMeasurements.START_OFFSET_Y])
    return np.linalg.norm(p - start_center)

def get_distance_from_exc(p: np.array):
    start_center = np.array([ZoneMeasurements.EXC_OFFSET_X, ZoneMeasurements.EXC_OFFSET_Y])
    return np.linalg.norm(p - start_center)

def get_distance_from_berm(p: np.array):
    start_center = np.array([ZoneMeasurements.BERM_OFFSET_X, ZoneMeasurements.BERM_OFFSET_Y])
    return np.linalg.norm(p - start_center)

def point_to_shapely(point: Point) -> shp.Point:
    return shp.Point(point.x, point.y)

def zone_to_poly(zone: Zone):
    return shp.Polygon(shell=[point_to_shapely(zone.v1), point_to_shapely(zone.v2), point_to_shapely(zone.v3), point_to_shapely(zone.v4)])

class ZonesNode(Node):
    def __init__(self):
        super().__init__("zones_node")

        ns = self.get_namespace().lstrip('/')
        self.frame = "map"
        if len(ns) != 0:
            self.frame = f"{ns}/{self.frame}"
    
        self.exc_zone_marker_pub = self.create_publisher(Marker, "exc_zone_marker", 10)
        self.berm_zone_marker_pub = self.create_publisher(Marker, "berm_zone_marker", 10)
        self.start_zone_marker_pub = self.create_publisher(Marker, "start_zone_marker", 10)

        self.exc_zone_pub = self.create_publisher(Zone, "exc_zone", 10)
        self.berm_zone_pub = self.create_publisher(Zone, "berm_zone", 10)
        self.start_zone_pub = self.create_publisher(Zone, "start_zone", 10)

        self.start_time = time.perf_counter_ns()
        self.create_timer(1 / 1.0, self.mainloop)

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
        zone_marker.header.frame_id = self.frame

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
        if rclpy.ok():
            if (exc_zone is not None):
                self.visualize_zone(exc_zone, self.exc_zone_marker_pub)
                self.exc_zone_pub.publish(exc_zone)

            if (berm_zone is not None):
                self.visualize_zone(berm_zone, self.berm_zone_marker_pub)
                self.berm_zone_pub.publish(berm_zone)

            if (start_zone is not None):
                self.visualize_zone(start_zone, self.start_zone_marker_pub)
                self.start_zone_pub.publish(start_zone)

def main():
    rclpy.init()
    zones_node = ZonesNode()

    rclpy.spin(zones_node)

    zones_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
