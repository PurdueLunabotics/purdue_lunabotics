#!/usr/bin/python3

from rclpy.node import Node
import rclpy


# find ground plane through 3 point or ground plane regression
# find whatever is below the ground plane and cluster them
# do circle regression off of some points above a certain layer

# find the point where the planes overlap????
# find normal vector there
# do the rotation of planes crap with the normal vectors ????

from std_msgs.msg import Int8, Int32, Bool, Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 # actually allows to read the point clouds
# import open3d as o3d
import numpy as np

from sklearn.linear_model import RANSACRegressor

import hough
from tf2_ros import TransformBroadcaster


class CraterGeneration(Node):
    """
    Generates the full image of a crater from the outer circle and rtab map.
    """

    def __init__(self, **kwargs):
        super().__init__('crater_generation_node', **kwargs)
        rclpy.get_global_executor().add_node(self)
        self.pointCloud = PointCloud2()
        self.ground = PointCloud2()
        self.ground_planes = PointCloud2()
        self.craters = [[], [], [], []]
        self.coeff = []
        
        ns = self.get_namespace().lstrip('/').split("/")[0]
        self.get_logger().warn(ns)
        if len(ns) != 0:
            ns = ns + '/'

        # transform we're looking for is from base link back to map
        self.map_used = f"{ns}map"
        
        self.crater_publisher = self.create_publisher(
            PointCloud2, "crater", 10
        )
        
        self.plane_publisher = self.create_publisher(
            PointCloud2, "ground_plane", 10
        )
        
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2, "rtabmap/cloud_obstacles", self.set_points,  10
        )
        self.ground_subscriber = self.create_subscription(
            PointCloud2, "rtabmap/cloud_ground", self.set_ground, 10
        )

        
        
        # only for bags
        # self.pointcloud_subscriber = self.create_subscription(
        #     PointCloud2, "/rtabmap/obstacles", self.set_points,  10
        # )
        # self.ground_subscriber = self.create_subscription(
        #     PointCloud2, "/rtabmap/ground", self.set_ground, 10
        # )


        self.create_timer(2, self.plane_generation)
        self.create_timer(1, self.estimate_crater)
        
        
        
    def plane_generation(self):
        ground_trainxy = []
        ground_trainz = []
        plane_pointcloud = []
        
        # self.get_logger().info(f"1{self.get_clock().now()}")
        # print("help me")
        stahp = False
        
        try:
            ground = point_cloud2.read_points(self.ground, field_names = ("x", "y", "z"), skip_nans=True)
            for p in ground: 
                ground_trainxy.append([p[0], p[1]]) 
                ground_trainz.append([p[2]])
                #self.get_logger().info(f"{p[0]}, {p[1]}, {p[2]}")
        except Exception as inst:
            self.get_logger().info(f"{inst}")
            self.get_logger().warn("Failed to read planes")
            stahp = True
        
        if(not stahp):
            # try:
            #     #self.get_logger().info(f"{ground_trainz}")
            # self.get_logger().info(f"2{self.get_clock().now()}")

            regressor = RANSACRegressor()

            
            # #regressor = RANSAC(model=LinearRegressor(), loss=square_error_loss, metric=mean_square_error)
        
            # self.get_logger().info(f"3{self.get_clock().now()}")

        
            regressor.fit(ground_trainxy, ground_trainz)
            # # except Exception as inst:
            # #     self.get_logger().info(f"{inst}")
            # #     self.get_logger().info(f"it killed itself")
        
            # self.get_logger().info(f"4{self.get_clock().now()}")
        
            # # the plane equation
            # self.get_logger().info(f"5{regressor.estimator_.coef_}")
            # self.get_logger().info(f"5{regressor.estimator_.intercept_}")
            self.coeff = [regressor.estimator_.coef_[0][0],regressor.estimator_.coef_[0][1], regressor.estimator_.intercept_[0]]
            pred_z = regressor.predict(ground_trainxy)

            # self.get_logger().info(f"{len(pred_z)}")

            # self.get_logger().info(f"{len(ground_trainxy)}")
            if (len(ground_trainxy) == len(pred_z)):
                for p in range(len(ground_trainxy)):
                    plane_pointcloud.append([ground_trainxy[p][0], ground_trainxy[p][1], pred_z[p][0]])
            # self.get_logger().warn("it do thing")
            # self.get_logger().warn("pointcloud")
            # self.get_logger().warn(f"{plane_pointcloud}")

        else: 
            print("failed to read pointcloud")
            
        header = Header()
        t = self.get_clock().now()
        header.stamp = t.to_msg()
        self.get_logger().warn(self.map_used)
        header.frame_id = self.map_used
        pc2 = point_cloud2.create_cloud_xyz32(header, plane_pointcloud)

        self.ground_planes = pc2
        self.plane_publisher.publish(pc2)


        
    def estimate_crater(self):
    
        did_read = True
        obst = []
        try:
            #self.get_logger
            vals = point_cloud2.read_points(self.pointCloud, field_names = ("x", "y", "z"), skip_nans=True)
            for p in vals:
                obst.append([p[0], p[1], p[2]]) 
        except Exception as inst:
            self.get_logger().info(f"{inst}")
            #self.get_logger().info(f"{self.pointcloud_subscriber.topic_name}")
            self.get_logger().info("first problem :D")
            did_read = False
        # get average height of ground
        # find points below average height of ground in obstacles
        #self.get_logger().info(f"{len(self.ground_planes.data)}")
        
        ground_vals = []
        try:
            ground = point_cloud2.read_points(self.ground_planes, field_names = ("x", "y", "z"), skip_nans=True)
            for p in ground: 
                ground_vals.append([p[0], p[1], p[2]]) 

            
            # self.get_logger().info(values)
            # self.get_logger().info(f"{ground_height}")
        except Exception as inst:
            self.get_logger().info(f"{inst}")
            self.get_logger().warn("Failed to read ground points")
            did_read = False
        
        crater_vals = []
        if(did_read):
            for p in obst:
                if (self.coeff[0]*p[0]+self.coeff[1]*p[1]+self.coeff[2] > p[2] + 0.06):
                # if(p[2]<ground_height-0.02):
                    crater_vals.append(p[:-1])
                    # self.get_logger().info(f"{p}")
                    
            # self.get_logger().info(f"{craternp}")
            # initial guess for the ring center and radius (if no previous info about those, increase uncertainty accordingly)
            guessed_cx = 0
            guessed_cy = 0
            guessed_r = 0.3

            # uncertainty of the initial guess
            uncertainty_pos = 10
            uncertainty_r = 0.1

            # width where points can still be counted to be part of the ring
            epsilon = 0.05
            
            crater_pointcloud = []
            
            for n in range(4):
                
                craternp = np.array(crater_vals, dtype=object)
            
                hough_cx, hough_cy, hough_r = hough.hough_pointcloud(
                    guessed_cx, guessed_cy, guessed_r, craternp,
                    uncertainty_pos,uncertainty_r, epsilon
                    )
                
                
                # try:
                    
                #     # self.get_logger().warn(f"hough {hough_cx}")
                #     # self.get_logger().warn(f"hough {hough_cy}")
                #     # self.get_logger().warn(f"hough {hough_r}")
                    
                    
                # except Exception as inst:
                #     self.get_logger().info(f"{inst}")   
                #     self.get_logger().warn("Not generating craters")
                
                def remove(j):
                    return(((j[0]>hough_cx-hough_r-0.075) and (j[0]<hough_cx+hough_r+0.075)) and ((j[1]>hough_cy-hough_r-0.075) and (j[1]<hough_cy+hough_r+0.075)))
                    
                    
                points_in = len(list(filter(remove,crater_vals)))
                # hough_r < 0.4 and
                if( hough_r > 0 and points_in > 13):
                    #self.get_logger().warn("god help")
                    for i in range(30):
                        x = hough_cx + hough_r * np.cos(i*12*2*np.pi/360)
                        y = hough_cy + hough_r * np.sin(i*12*2*np.pi/360)
                        crater_pointcloud.append([x, y, 0])
                        
                        
                    def dont_remove(j):
                        return(((j[0]<hough_cx-hough_r-0.075) or (j[0]>hough_cx+hough_r+0.075)) or ((j[1]<hough_cy-hough_r-0.075) or (j[1]>hough_cy+hough_r+0.075)))
                    
                    crater_vals = list(filter(dont_remove, crater_vals))
                    
                    # self.get_logger().info(f"{crater_vals}")
                
            
            header = Header()
            t = self.get_clock().now()
            header.stamp = t.to_msg()
            header.frame_id = self.map_used
            pc2 = point_cloud2.create_cloud_xyz32(header, crater_pointcloud)

            self.crater_publisher.publish(pc2)
        
        
    def set_points(self, points : PointCloud2):
        # self.get_logger().info("help me go]d")
        self.pointCloud = points
        
    def set_ground(self, ground : PointCloud2):
        # self.get_logger().info("I hate it here")
        self.ground = ground

def main():
    rclpy.init()
    node = CraterGeneration()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()