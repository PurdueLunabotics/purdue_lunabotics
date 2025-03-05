import rospy
from rclpy.node import Node

import cv2
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2

from cv_bridge import CvBridge

import open3d as o3d

import torch

class SlamPCPublisher:
    def __init__(self):
        rospy.init_node("slam_pc_publisherx")

        self.bridge = CvBridge()

        # topic definitions
        self.depth_topic = rospy.resolve_name("/depth/image_rect_raw")
        self.camera_info_topic = rospy.resolve_name("/depth/camera_info")
        self.pc_pub_topic = rospy.resolve_name("/pc")

        # adjustable parameter definitions
        self.voxel_size = rospy.get_param("~voxel_size")
        self.voxelize = rospy.get_param("~voxelization")

        # subscribers
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.image_callback, queue_size=10)
        self.cam_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=10)

        # publishers
        self.pc_pub = self.create_publisher(PointCloud2, self.pc_pub_topic, 10)

        self.height = 0
        self.width = 0

        self.fx = 0
        self.fy = 0

        self.cx = 0
        self.cy = 0

        rospy.spin()

    # ==========================================================================================================
    # UTIL METHODS
    # ==========================================================================================================
    
    def depth_map_to_points(
        self,
        depth_map: np.ndarray,
    ) -> o3d.geometry.PointCloud:
        """
        Converts depth map numpy array into points and colors np arrays.

        Args:
            depth_map (np.array): depth numpy array
            imgL (np.array): left image in grayscale
            origin_pos (np.array): position of the camera in [x, y, z]
            origin_rpy (list-like): origin orientation of the camera in [roll, pitch, yaw]
        Return:
            points, colors - list-like, numpy arrays of matching points and colors
        """

        depth_map = torch.from_numpy(depth_map)  # convert depth map into tensor space
        imgL = (
            torch.from_numpy(imgL).float() / 255.0
        )  # convert image to tensor space and normalize image to within 0 to 1 pixel intensity

        cols, rows = torch.meshgrid(
            torch.arange(depth_map.shape[0]),
            torch.arange(depth_map.shape[1]),
            indexing="ij",
        )

        mask = depth_map != 0
        depth_map = depth_map[mask]
        x_indices = rows[mask]
        y_indices = cols[mask]

        Z = depth_map
        X = (x_indices - self.cx) * depth_map / self.fx
        Y = (y_indices - self.cy) * depth_map / self.fy
        points = torch.stack([X, Y, Z], dim=1)

        points = points.numpy()

        return points
    
    def compute_point_cloud_from_depth(
        self,
        # imgL: np.ndarray,
        depth_map: np.ndarray
    ):
        """
        Compute point cloud from stereo image input using Semi-Global Block Matching disparity calculation method.

        Args:
            imgL: grayscale left image
            imgR: grayscale right image
            origin_pos (list-like): camera position (x, y, z)
            origin_rpy (list-like): camera orientation (roll, pitch, yaw).
            disparity_method (str): allows for distinction between block matching and semi-global block matching
        Return:
            pc (open3d.geometry.PointCloud): point cloud of stereo image data
        """

        # profiler = cProfile.Profile()
        # profiler.enable()

        # imgL = cv2.cvtColor(
        #     imgL,
        #     # convert grayscale opencv image to RGB for point cloud color assignment
        #     cv2.COLOR_GRAY2RGB,
        # )

        points = self.depth_map_to_points(depth_map)

        # convert to open3d point cloud for downsampling
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.Vector3dVector(points)

        if self.voxelize:
          # downsample to voxel size of 2cm
          o3d_pc = o3d_pc.voxel_down_sample(voxel_size=self.voxel_size)

        return o3d_pc
    
    def o3d_pc_to_point_cloud2(self, header, o3d_pc) -> PointCloud2:
        """
        Converts an Open3D point cloud to a ROS2 PointCloud2 message.

        Args:
            stamp_msg (Header msg): timestamp as a header message
            o3d_pc (o3d.geometry.PointCloud): pointcloud to convert
        Return:
            PointCloud2 message representing the o3d point cloud.
        """

        points = np.asarray(o3d_pc.points, dtype=np.float32)
        # colors = np.asarray(o3d_pc.colors, dtype=np.float32)

        # msg = PointCloud2()
        # msg.header.stamp = self.get_clock().now().to_msg()
        msg = pc2.create_cloud_xyz32(header, points)

        # msg.height = 1
        # msg.width = points.shape[0]
        # msg.is_dense = True  # no invalid values
        # msg.is_bigendian = False
        # msg.point_step = 16  # 16 bytes per point
        # msg.row_step = msg.point_step * points.shape[0]  # byte length of row

        # msg.fields = [
        #     PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        #     PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        #     PointField(
        #         name="rgb", offset=12, datatype=PointField.UINT32, count=1
        #     ),
        # ]

        # # convert colors from 0-1 RGB to 0-1 intensity
        # colors = colors * 255
        # intensity = np.dot(colors, [0.2989, 0.5870, 0.1140]).astype(np.float32)

        # # make numpy structured array of points
        # structured_data = np.zeros(
        #     points.shape[0],
        #     dtype=[
        #         ("x", np.float32),
        #         ("y", np.float32),
        #         ("z", np.float32),
        #         ("intensity", np.uint32),
        #     ],
        # )
        # structured_data["x"] = points[:, 0]  # populate x-coords
        # structured_data["y"] = points[:, 1]  # populate y coords
        # structured_data["z"] = points[:, 2]  # populate z coords

        # # populate intensity vals with 32-bit ints
        # structured_data["intensity"] = intensity.flatten()

        # msg.data = structured_data.tobytes()  # reads byte data when processing pc
        return msg

    # =========================================================================================================
    # CALLBACKS
    # =========================================================================================================

    def camera_info_callback(self, msg):
        if msg is not None and self.fx != 0 and self.fy != 0 and self.cx != 0 and self.cy != 0:
          self.height = msg.height
          self.width = msg.width

          self.fx = msg.K[0]
          self.fy = msg.K[4]

          self.cx = msg.K[2]
          self.cy = msg.K[5]

    def image_callback(self, msg):
        if msg is not None:
            encoding = msg.encoding

            # expecting grayscale images
            depth = self.bridge.imgmsg_to_cv2(msg.image_pair.right, desired_encoding=encoding)
            depth.header = msg.header

            if msg is not None:
                # pc publishing -------- (USED WITH SLAM LOOP CLOSURE)
                pc = self.compute_point_cloud_from_depth(depth)
                pc_msg = self.o3d_pc_to_point_cloud2(depth.header, pc)

                self.pc_pub.publish(pc_msg)


def main():
    node = SlamPCPublisher()

if __name__ == "__main__":
    main()
