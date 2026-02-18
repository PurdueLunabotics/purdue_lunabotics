#!/usr/bin/python3

from rclpy.node import Node
import rclpy


# find ground plane through 3 point or ground plane regression
# find whatever is below the ground plane and cluster them
# do circle regression off of some points above a certain layer

from std_msgs.msg import Int8, Int32, Bool, Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 # actually allows to read the point clouds
# import open3d as o3d
import numpy as np

import hough

class CraterGeneration(Node):
    """
    Generates the full image of a crater from the outer circle and rtab map.
    """

    def __init__(self, **kwargs):
        super().__init__('crater_generation_node', **kwargs)
        rclpy.get_global_executor().add_node(self)
        self.pointCloud = PointCloud2()
        self.ground = PointCloud2()
        
        
        self.crater_publisher = self.create_publisher(
            PointCloud2, "crater", 10
        )
        
        # self.pointcloud_subscriber = self.create_subscription(
        #     PointCloud2, "d455_front/points", self.set_points,  10
        # )
        
        
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2, "/rtabmap/cloud_obstacles", self.set_points,  10
        )
        self.ground_subscriber = self.create_subscription(
            PointCloud2, "/rtabmap/cloud_ground", self.set_ground, 10
        )

        self.create_timer(1, self.estimate_crater)

        
    def estimate_crater(self):
    
        obst = []
        try:
            #self.get_logger
            vals = point_cloud2.read_points(self.pointCloud, field_names = ("x", "y", "z"), skip_nans=True)
            for p in vals:
                obst.append((p[0], p[1], p[2])) 
        except Exception as inst:
            self.get_logger().info(f"{inst}")
            self.get_logger().info("first problem :D")
        # get average height of ground
        # find points below average height of ground in obstacles
        self.get_logger().info(f"{len(self.ground.data)}")
        
        ground_vals = []
        try:
            ground = point_cloud2.read_points(self.ground, field_names = ("x", "y", "z"), skip_nans=True)
            for p in ground: 
                ground_vals.append((p[0], p[1], p[2])) 
            ground_height = np.mean(ground_vals, axis = 0)[2]
        
            # self.get_logger().info(values)
            self.get_logger().info(f"{ground_height}")
        except Exception as inst:
            self.get_logger().info(f"{inst}")
            self.get_logger().warn("Failed to read")
        
        crater_vals = []
        
        for p in obst:
            if(p[2]<ground_height-0.01):
                crater_vals.append(p[:-1])
                
        for i in range(30):
                x = 0.3 + 0.25 * np.cos(i*12*2*np.pi/360)
                y = 0.2 + 0.25 * np.sin(i*12*2*np.pi/360)
                crater_vals.append([x, y, 0])
        
        for i in range(30):
                x = 2 + 0.25 * np.cos(i*12*2*np.pi/360)
                y = 0.2 + 0.25 * np.sin(i*12*2*np.pi/360)
                crater_vals.append([x, y, 0])
                
        
        
        
        
        # self.get_logger().info(f"{craternp}")
        # initial guess for the ring center and radius (if no previous info about those, increase uncertainty accordingly)
        guessed_cx = 0
        guessed_cy = 0
        guessed_r = 0.3

        # uncertainty of the initial guess
        uncertainty = 10

        # width where points can still be counted to be part of the ring
        epsilon = 0.05
        
        crater_pointcloud = []
        
        for n in range(4):
            
            craternp = np.array(crater_vals)
        
            hough_cx, hough_cy, hough_r = hough.hough_pointcloud(
                guessed_cx, guessed_cy, guessed_r, craternp,
                uncertainty, epsilon
                )
            
            try:
                
                self.get_logger().warn(f"hough {hough_cx}")
                self.get_logger().warn(f"hough {hough_cy}")
                self.get_logger().warn(f"hough {hough_r}")
                
                
            except Exception as inst:
                self.get_logger().info(f"{inst}")   
                self.get_logger().warn("Not generating craters")
                
           
            
            if(hough_r < 0.4 and hough_r > 0):
                for i in range(30):
                    x = hough_cx + hough_r * np.cos(i*12*2*np.pi/360)
                    y = hough_cy + hough_r * np.sin(i*12*2*np.pi/360)
                    crater_pointcloud.append([x, y, 0])
                    
                    
                def dont_remove(j):
                    return(((j[0]<hough_cx-hough_r-0.05) or (j[0]>hough_cx+hough_r+0.05)) or ((j[1]<hough_cy-hough_r-0.05) or (j[1]>hough_cy+hough_r+0.05)))
                
                crater_vals = list(filter(dont_remove, crater_vals))
                
                self.get_logger().info(f"{crater_vals}")
            
        
        header = Header()
        t = self.get_clock().now()
        header.stamp = t.to_msg()
        header.frame_id = "map"
        pc2 = point_cloud2.create_cloud_xyz32(header, crater_pointcloud)

        self.crater_publisher.publish(pc2)
        
        
    def set_points(self, points : PointCloud2):
        self.get_logger().info("help me go]d")
        self.pointCloud = points
        
    def set_ground(self, ground : PointCloud2):
        self.get_logger().info("I hate it here")
        self.ground = ground

def main():
    rclpy.init()
    node = CraterGeneration()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()