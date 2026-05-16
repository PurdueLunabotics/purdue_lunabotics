#!/usr/bin/python3

from rclpy.node import Node
import rclpy


# find ground plane through 3 point or ground plane regression
# find whatever is below the ground plane and cluster them
# do circle regression off of some points above a certain layer

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 # actually allows to read the point clouds
# import open3d as o3d
import numpy as np

# from sklearn.linear_model import RANSACRegressor

import hough

# Disable these if costmap isn't worth it
from tf2_ros import Time, TransformBroadcaster, TransformListener, Buffer, TransformStamped
from lunabot_behavior.states.init import tf_to_matrix
from rclpy.task import Future 
import time
from lunabot_behavior.zones import bounding_box, ZoneMeasurements
from operator import itemgetter

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
        self.craters = []
        self.coeff = []
        self.craters = []
        
        ns = self.get_namespace().lstrip('/').split("/")[0]
        #self.get_logger().warn(ns)
        if len(ns) != 0:
            ns = ns + '/'

        # transform we're looking for is from base link back to map
        self.map_used = f"mini/map"
        
        self.crater_publisher = self.create_publisher(
            PointCloud2, "crater", 10
        )

        self.cratervals_publisher = self.create_publisher(
            PointCloud2, "cratervals", 10
        )
        
        self.plane_publisher = self.create_publisher(
            PointCloud2, "ground_plane", 10
        )
        
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2, "mini/rtabmap/cloud_obstacles", self.set_points,  10
        )
        self.ground_subscriber = self.create_subscription(
            PointCloud2, "mini/rtabmap/cloud_ground", self.set_ground, 10
        )

        self.bounding_pub = self.create_publisher(
            PointCloud2, "boundingkrill", 10
        )
        
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.crater_clear_counter = 0
        
        
        # only for bags
        # self.pointcloud_subscriber = self.create_subscription(
        #     PointCloud2, "/rtabmap/obstacles", self.set_points,  10
        # )
        # self.ground_subscriber = self.create_subscription(
        #     PointCloud2, "/rtabmap/ground", self.set_ground, 10
        # )


        # self.create_timer(1, self.plane_generation)
        self.create_timer(1, self.estimate_crater)
        
        
        
    # def plane_generation(self):
    #     ground_trainxy = []
    #     ground_trainz = []
    #     plane_pointcloud = []
        
    #     # self.get_logger().info(f"1{self.get_clock().now()}")
    #     # print("help me")
    #     stahp = False
        
    #     try:
    #         ground = point_cloud2.read_points(self.ground, field_names = ("x", "y", "z"), skip_nans=True)
    #         for p in ground: 
    #             ground_trainxy.append([p[0], p[1]]) 
    #             ground_trainz.append([p[2]])
    #             #self.get_logger().info(f"{p[0]}, {p[1]}, {p[2]}")
    #     except Exception as inst:
    #         self.get_logger().info(f"{inst}")
    #         self.get_logger().warn("Failed to read planes")
    #         stahp = True

    #     if len(ground_trainxy) == 0 or len(ground_trainz) == 0:
    #         stahp = True
        
    #     if(not stahp):
    #         # try:
    #         #     #self.get_logger().info(f"{ground_trainz}")
    #         # self.get_logger().info(f"2{self.get_clock().now()}")

    #         regressor = RANSACRegressor()

            
    #         # #regressor = RANSAC(model=LinearRegressor(), loss=square_error_loss, metric=mean_square_error)
        
    #         # self.get_logger().info(f"3{self.get_clock().now()}")

        
    #         regressor.fit(ground_trainxy, ground_trainz)
    #         # # except Exception as inst:
    #         # #     self.get_logger().info(f"{inst}")
    #         # #     self.get_logger().info(f"it killed itself")
        
    #         # self.get_logger().info(f"4{self.get_clock().now()}")
        
    #         # # the plane equation
    #         # self.get_logger().info(f"5{regressor.estimator_.coef_}")
    #         # self.get_logger().info(f"5{regressor.estimator_.intercept_}")
    #         self.coeff = [regressor.estimator_.coef_[0][0],regressor.estimator_.coef_[0][1],regressor.estimator_.intercept_[0]]
    #         pred_z = regressor.predict(ground_trainxy)

    #         # self.get_logger().info(f"{len(pred_z)}")

    #         # self.get_logger().info(f"{len(ground_trainxy)}")
    #         if (len(ground_trainxy) == len(pred_z)):
    #             for p in range(len(ground_trainxy)):
    #                 plane_pointcloud.append([ground_trainxy[p][0], ground_trainxy[p][1], pred_z[p][0]])
    #         # self.get_logger().warn("it do thing")
    #         # self.get_logger().warn("pointcloud")
    #         # self.get_logger().warn(f"{plane_pointcloud}")

    #     else: 
    #         print("failed to read pointcloud")
            
    #     header = Header()
    #     t = self.get_clock().now()
    #     header.stamp = t.to_msg()
    #     #self.get_logger().warn(self.map_used)
    #     header.frame_id = self.map_used
    #     pc2 = point_cloud2.create_cloud_xyz32(header, plane_pointcloud)

    #     self.ground_planes = pc2
    #     self.plane_publisher.publish(pc2)


    #     # self.costmap_client.wait_for_service()
    #     # self.costmap_client.call_async(GetCostmap.Request()).add_done_callback(self.costmap_cb)


        
    def estimate_crater(self):
        # self.costmap_client.wait_for_service()
        # self.costmap_client.call_async(GetCostmap.Request()).add_done_callback(self.costmap_cb)

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
        # self.get_logger().info(f"{len(self.ground_planes.data)}")
        
        # ground_vals = []
        # try:
        #     ground = point_cloud2.read_points(self.ground_planes, field_names = ("x", "y", "z"), skip_nans=True)
        #     for p in ground: 
        #         ground_vals.append([p[0], p[1], p[2]]) 

            
        #     # self.get_logger().info(values)
        #     # self.get_logger().info(f"{ground_height}")
        # except Exception as inst:
        #     self.get_logger().info(f"{inst}")
        #     self.get_logger().warn("Failed to read ground points")
        #     did_read = False
        
        # if(self.coeff == []):
        #     did_read = False

        crater_vals = []
        crater_check = []


        if(did_read):
            # for p in obst:
            #     if ((self.coeff[0]*p[0]+self.coeff[1]*p[1]+self.coeff[2]) > p[2] + 0.09): # and self.is_blocked(self.costmap, p[0], p[1], 252)):
            #     # if(p[2]<ground_height-0.02):
            #         crater_vals.append(p[:-1])
            #         crater_check.append(p)
            #         # self.get_logger().info(f"{p}")


            start_zone = [ZoneMeasurements.START_OFFSET_X-ZoneMeasurements.START_LENGTH_X/2, ZoneMeasurements.START_OFFSET_Y-ZoneMeasurements.START_LENGTH_Y/2, ZoneMeasurements.START_OFFSET_X+ZoneMeasurements.START_LENGTH_X/2, ZoneMeasurements.START_OFFSET_Y+ZoneMeasurements.START_LENGTH_Y/2]
            
            self.get_logger().warn(f"{start_zone[0]}")
            self.get_logger().warn(f"{start_zone[2]}")
            self.get_logger().warn(f"{start_zone[1]}")
            self.get_logger().warn(f"{start_zone[3]}")
            # self.get_logger().warn(f"{np.mean(obst[:][0])}")
            # self.get_logger().warn(f"{np.mean(obst[:][1])}")

            points = []

            for y in np.arange(bounding_box[1], bounding_box[3], 0.03):
                points.append([bounding_box[0], y, 0])
                points.append([bounding_box[2], y, 0])

            for x in np.arange(bounding_box[0], bounding_box[2], 0.03):
                points.append([x, bounding_box[1], 0])
                points.append([x, bounding_box[3], 0])
            
            cloud = point_cloud2.create_cloud_xyz32(Header(frame_id=self.map_used, stamp=self.get_clock().now().to_msg()), points)
            self.bounding_pub.publish(cloud)

            is_mirrored = ZoneMeasurements.BERM_OFFSET_X > ZoneMeasurements.EXC_OFFSET_X

            # maybe do voxelization??? idk seems a bit late for that atp
            groups = np.arange(min(obst, key=itemgetter(2))[2],max(obst, key=itemgetter(2))[2], 0.03)
                
                

            for p in obst:
                if ((p[0]>bounding_box[0]+0.2 and (p[0]<bounding_box[2]-0.2) and (p[1]>bounding_box[1]+0.2 and p[1]<bounding_box[3]-0.2)) and (((p[0]<start_zone[0] - 0.05 or  p[1]<start_zone[1] - 0.05) and (not is_mirrored)) or ((p[0]>start_zone[2]+0.05 or  p[1]<start_zone[1]-0.05) and is_mirrored))): # and self.is_blocked(self.costmap, p[0], p[1], 252)):
                # if(p[2]<ground_height-0.02):
                    crater_vals.append(p[:-1])

                    crater_check.append(p)
                    
                # else:
                    # self.get_logger().info("found bad point")
                        
                    # self.get_logger().info(f"{p}")
            
            # map_to_main_tag_tf = tf_to_matrix(self.tf_buf.lookup_transform("mini/map", "main_deposition", Time()))
            # main_tag_to_base_tf = tf_to_matrix(self.tf_buf.lookup_transform("deposition_apriltag_optical_frame", "base_link", Time()))
            # tf_matrix = tf_transformations.concatenate_matrices(map_to_main_tag_tf, main_tag_to_base_tf)


            # WIDTH = 1.0
            # LENGTH = 0.7

            # point = np.zeros(4)
            # point[3] = 1.0

            # main_box = []
            
            # for y in np.arange(-LENGTH/2, LENGTH/2, 0.03):
            #     for x in np.arange(-WIDTH/2, WIDTH/2, 0.03):
            #         point[0] = x
            #         point[1] = y
            #         new_point = tf_matrix @ point
            #         main_box.append([new_point[0], new_point[1], 0])
            
            if len(crater_vals) != 0:
                pc2 = point_cloud2.create_cloud_xyz32(Header(frame_id=self.map_used, stamp=self.get_clock().now().to_msg()), crater_check)

                self.cratervals_publisher.publish(pc2)
                    
            # self.get_logger().info(f"{craternp}")
            # initial guess for the ring center and radius (if no previous info about those, increase uncertainty accordingly)
            guessed_cx = 0
            guessed_cy = 0
            guessed_r = 0.15

            # uncertainty of the initial guess
            uncertainty_pos = 5
            uncertainty_r = 0.1

            # width where points can still be counted to be part of the ring
            epsilon = 0.04
            
            if self.crater_clear_counter > 3:
                self.craters = self.craters[(5*30):]
            

            # is it better to increase and generate more obstacles, with the risk of having important obstacles be generated later?
            for _ in range(10):
                
                #self.get_logger().info(f"{self.get_clock().now()}")

                print(f"starting time {time.time()}")
                craternp = np.array(crater_vals, dtype=object)
                didRun = False
                if crater_vals != []:
                    didRun = True
                    hough_cx, hough_cy, hough_r = hough.hough_pointcloud(
                        guessed_cx, guessed_cy, guessed_r, craternp,
                        uncertainty_pos,uncertainty_r, epsilon
                        )
                
                
                if didRun:
                # try:
                    
                    self.get_logger().warn(f"hough {hough_cx}")
                    self.get_logger().warn(f"hough {hough_cy}")
                    self.get_logger().warn(f"hough {hough_r}")
                        
                        
                    #self.get_logger().info(f"3{self.get_clock().now()}")    
                    # except Exception as inst:
                    #     self.get_logger().info(f"{inst}")   
                    #     self.get_logger().warn("Not generating craters")
                    
                    def remove(j):
                        return(((j[0]>hough_cx-hough_r-0.10) and (j[0]<hough_cx+hough_r+0.1)) and ((j[1]>hough_cy-hough_r-0.10) and (j[1]<hough_cy+hough_r+0.10)))
                        
                        
                    points_in = len(list(filter(remove,crater_vals)))
                    # hough_r < 0.4 and
                    if(hough_r > 0 and points_in > 15 and points_in < 100):
                        #self.get_logger().warn("god help")
                        i = np.linspace(0,30,30)
                        x = hough_cx + hough_r * np.cos(i*12*2*np.pi/360)
                        y = hough_cy + hough_r * np.sin(i*12*2*np.pi/360)
                        
                        # self.get_logger().info(f"{np.stack((x,y,np.linspace(0,0,30)), axis= -1)}")
                        # self.get_logger().info(f"{crater_pointcloud}")
                        if  len(self.craters) == 0:
                            self.craters = np.stack((x,y,np.linspace(0,0,30)), axis= -1)
                            # self.get_logger().info(f"trying my best {crater_pointcloud}")
                        else:
                            self.craters = np.concatenate((self.craters, np.stack((x,y,np.linspace(0,0,30)), axis= -1)))
                            # self.get_logger().info(f"can you work pls {crater_pointcloud}")

                            
                            
                        def dont_remove(j):
                            return(((j[0]<hough_cx-hough_r-0.25) or (j[0]>hough_cx+hough_r+0.25)) or ((j[1]<hough_cy-hough_r-0.25) or (j[1]>hough_cy+hough_r+0.25)))
                        
                        crater_vals = list(filter(dont_remove, crater_vals))
                        pc2 = point_cloud2.create_cloud_xyz32(Header(frame_id=self.map_used, stamp=self.get_clock().now().to_msg()), self.craters)
                        self.crater_publisher.publish(pc2)
                        
                        # self.get_logger().info(f"{crater_vals}")
                    
                
                    if (points_in > 100 or points_in < 5):
                        def dont_remove_walls(j):
                            return(((j[0]<hough_cx-hough_r-0.05) or (j[0]>hough_cx+hough_r+0.05)) or ((j[1]<hough_cy-hough_r-0.05) or (j[1]>hough_cy+hough_r+0.05)))
                        
                        crater_vals = list(filter(dont_remove_walls, crater_vals))
            self.crater_clear_counter += 1
            pc2 = point_cloud2.create_cloud_xyz32(Header(frame_id=self.map_used, stamp=self.get_clock().now().to_msg()), self.craters)
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