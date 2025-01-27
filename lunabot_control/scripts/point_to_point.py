#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, Point
from pid_controller import PIDController
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import numpy as np


class PointToPoint:
    def __init__(self):
        rospy.init_node("point_to_point_node")

        self.LINEAR_P = 5.0
        self.LINEAR_I = 0
        self.LINEAR_D = 0
        self.LINEAR_TOLERANCE = 0.2  # meters
        self.MAX_LINEAR_SPEED = 0.5  # m/s
        self.linear_pid = PIDController(
            self.LINEAR_P,
            self.LINEAR_I,
            self.LINEAR_D,
            max_output=self.MAX_LINEAR_SPEED,
        )

        self.ANGULAR_P = 5.0
        self.ANGULAR_I = 0
        self.ANGULAR_D = 0
        self.ANGULAR_TOLERANCE_DEG = 5
        self.ANGULAR_TOLERANCE_RAD = np.deg2rad(self.ANGULAR_TOLERANCE_DEG)
        self.MAX_ANGULAR_SPEED_DEG_PER_SEC = 90
        self.MAX_ANGULAR_SPEED_RAD_PER_SEC = np.deg2rad(
            self.MAX_ANGULAR_SPEED_DEG_PER_SEC
        )
        self.angular_pid = PIDController(
            self.ANGULAR_P,
            self.ANGULAR_I,
            self.ANGULAR_D,
            max_output=self.MAX_ANGULAR_SPEED_RAD_PER_SEC,
        )

        self.robot_pose = [None, None, None]  # x, y, heading (rad)
        self.last_pose = [None, None, None]  # for velocity calculations

        self.FREQUENCY = 60.0
        self.pid_dt = 1 / self.FREQUENCY
        self.prev_pid_time = 0
        self.odom_dt = 1 / self.FREQUENCY
        self.prev_odom_time = 0

        self.angular_vel = 0
        self.linear_vel = 0

        self.odom_velocity = [None, None]

        self.at_angle_target = True
        self.at_linear_target = True

        self.prev_linear_error = float("inf")

        self.target_point_index = 0
        self.target_point = [None, None]
        self.path = []

        # PUBLISHERS ==================================================================================================
        cmd_vel_topic = rospy.get_param("/cmd_vel_topic", "/cmd_vel")
        self.cmd_vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # TODO: debugging
        self.angular_disparity_publisher = rospy.Publisher(
            "/ptp/angular_disparity", Float32, queue_size=10
        )
        self.linear_disparity_publisher = rospy.Publisher(
            "/ptp/linear_disparity", Float32, queue_size=10
        )
        self.heading_publisher = rospy.Publisher("/ptp/heading", Float32, queue_size=10)
        self.angle_target_publisher = rospy.Publisher(
            "/ptp/angle_target", Float32, queue_size=10
        )
        self.pid_linear_publisher = rospy.Publisher(
            "/ptp/pid_linear", Float32, queue_size=10
        )
        self.pid_angular_publisher = rospy.Publisher(
            "/ptp/pid_angular", Float32, queue_size=10
        )
        self.path_segment_publisher = rospy.Publisher(
            "/ptp/current_target", Marker, queue_size=10
        )
        self.path_publisher = rospy.Publisher("/ptp/line_path", Marker, queue_size=10)

        # SUBSCRIBERS ==================================================================================================
        odom_topic = rospy.get_param("/odom_topic", "/odom")
        rospy.Subscriber(odom_topic, Odometry, self.__odom_callback)

        path_topic = rospy.get_param("/nav/global_path_topic", "/nav/global_path")
        rospy.Subscriber(path_topic, Path, self.__path_callback)

    # ==================================================================================================================
    # CALLBACKS
    # ==================================================================================================================

    def __odom_callback(self, msg: Odometry):
        # self.robot_velocity = [msg.twist.twist.linear, msg.twist.twist.angular]
        angles = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )
        self.robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            angles[2], #-pi to pi
        )

        self.odom_dt = rospy.Time.now().to_sec() - self.prev_odom_time
        self.prev_odom_time = rospy.Time.now().to_sec()

        if (
            self.robot_pose != [None, None, None]
            and self.last_pose != [None, None, None]
            and self.odom_dt != 0
        ):
            if self.odom_dt == 0 or self.odom_dt is None:
                self.odom_dt = 1 / self.FREQUENCY  # ensure no div by 0 errors

            # linear velocity
            self.odom_velocity[0] = (
                np.linalg.norm(
                    np.array(self.robot_pose[:2]) - np.array(self.last_pose[:2])
                )
                / self.odom_dt
            )
            # angular velocity
            self.odom_velocity[1] = (
                self.robot_pose[2] - self.last_pose[2]
            ) / self.odom_dt

        # update last position
        self.last_pose = self.robot_pose

    def __path_callback(self, msg: Path):
        complex_path = []  # create list for storing all d* points
        for pose in msg.poses:
            # add all points to complex path
            complex_path.append((pose.pose.position.x, pose.pose.position.y))

        marker_points = []
        self.path, marker_points = self.__simplify_path(complex_path)

        # initialize target point
        self.target_point_index = 0
        self.target_point = self.path[self.target_point_index]

        # visualize sequence of lines
        self.__visualize_line_path(marker_points)

    # ==================================================================================================================
    # PATH PROCESSING
    # ==================================================================================================================


    ### Simplifies a complex path by removing points that are close to colinear with their neighbors
    ### ensures that gradual changes are still done

    def __simplify_path(self,points, difference_threshold=0.97):
        filtered_points = [points[0]]
        last_filtered_index = 0
        if len(points) >= 3:
            for i in range(2,len(points)):
                p1 = points[last_filtered_index]
                p2 = points[last_filtered_index+1]
                p3 = points[i]
                # find projection of vectors on each other
                #vector 1 is from the last filtered point to the next point in the complex path
                v1 = np.array(p2,'float64') - np.array(p1,'float64')
                v1 /= np.linalg.norm(v1)  # normalize
                #vector 2 is from the last filtered point to the current point in the complex path
                v2 = np.array(p3,'float64') - np.array(p1,'float64')
                v2 /= np.linalg.norm(v2)  # normalize
                print(str(v1)+" "+str(v2))
                
                # calculate projection size
                projection_size = np.dot(v1, v2)
                                
                # if within difference threshold, keep the point
                if projection_size <= difference_threshold:
                    j=0
                    #check the points between the indicies of p1 and p3 to find the last p2 that isn't within tolerance
                    for j in range(0,i-last_filtered_index):
                        p2 = points[last_filtered_index+1+j]
                        v1 = np.array(p2,'float64') - np.array(p1,'float64')
                        v1 /= np.linalg.norm(v1)  # normalize
                        v2 = np.array(p3,'float64') - np.array(p1,'float64')
                        v2 /= np.linalg.norm(v2)  # normalize
                        projection_size = np.dot(v1, v2)
                        if projection_size>difference_threshold:
                            p2 = points[last_filtered_index+j]
                            break
                    
                    filtered_points.append(p2)
                    last_filtered_index = last_filtered_index+j


                # add last point to end up in same location
                if i == len(points)-1:
                    filtered_points.append(points[-1])
        else: 
            filtered_points = points
        if len(filtered_points) == 0:
            filtered_points = points
        
        marker_points = self.__point_list_to_ros_point_list(filtered_points)

        marker_points.insert(0, marker_points[0])
        marker_points.insert(0, Point(self.robot_pose[0], self.robot_pose[1], 0))

        print("SIMPLIFY PATH: Points: "+str(points)+" ; Simple Points: "+str(filtered_points)) #put the points and filtered points into console
        return filtered_points, marker_points

    def __point_list_to_ros_point_list(self, points):
        results = []
        for i in range(len(points) - 1):
            results.append(Point(points[i][0], points[i][1], 0))
            results.append(Point(points[i + 1][0], points[i + 1][1], 0))

        return results

    # ==================================================================================================================
    # MOTION
    # ==================================================================================================================

    def __turn_to_point(self, point, pose):
        """
        Computes angular velocity required to turn toward target point and updates global angular velocity veriable.

        Args:
            point (list/array): target point in format [x (m), y (m)]
            pose (list/array): robot pose in format [x (m), y (m), heading (rad)]
        """
        # store heading for computation - resistant to changes in variable caused by odom callback during loop execution
        current_pose = pose
        # calculate angle to target from x axis
        pose_target_angle = np.arctan2(
            point[1] - current_pose[1], point[0] - current_pose[0]
        )
        # subtract heading to find angle error
        angle_error = pose_target_angle - current_pose[2]
        # check if around-the-world distance is smaller than current different
        if np.abs(2 * np.pi - angle_error) < np.abs(angle_error):
            # normalize to make error reflect around-the-world
            angle_error = 2 * np.pi - angle_error

        self.angular_disparity_publisher.publish(angle_error)
        print("P2P: Robot Angle: "+str(current_pose[2])+" Target Angle: "+str(pose_target_angle)+" Angular Error: "+str(angle_error))
        # check if robot heading is within tolerance - if so, terminate turning procedure
        self.at_angle_target = np.abs(angle_error) < self.ANGULAR_TOLERANCE_RAD

        if not self.at_angle_target and not self.at_linear_target: #only turn if not within linear target tolerance
            self.linear_vel = (
                0  # stop linear translation if angle error becomes too big
            )

            self.angular_vel = -self.angular_pid.calculate(
                state=angle_error, dt=self.pid_dt, setpoint=0
            )
            self.pid_angular_publisher.publish(self.angular_vel)
        else:
            self.angular_vel = 0

    def __translate_to_point(self, point, pose):
        """
        Translates robot toward target when heading is pointed toward target.

        Args:
            point (list/array): target point in format [x (m), y (m)]
            pose (list/array): robot pose in format [x (m), y (m), heading (rad)]
        """
        # store x and y coords of pose in a location variable
        current_pose = pose
        current_location = np.array(current_pose[:2])
        # calculate distance to target as error
        linear_error = np.linalg.norm(np.array(point[:2]) - current_location)

        # switch sign of the error if magnitude is increasing beyond tolerance - robot needs to go in negative direction
        if (
            np.abs(linear_error) - np.abs(self.prev_linear_error)
            >= self.LINEAR_TOLERANCE
        ):
            linear_error = linear_error * -1

        # update previous linear error after checking that the magnitude is decreasing
        self.prev_linear_error = linear_error

        # check if robot linear position is within tolerance - if so, terminate linear motion
        self.at_linear_target = np.abs(linear_error) < self.LINEAR_TOLERANCE

        if (not self.at_linear_target) and self.angular_vel == 0:  # only translate if stopped turning
            self.linear_vel = -self.linear_pid.calculate(state=linear_error, dt=self.pid_dt, setpoint=0)
        else:
            self.linear_vel = 0

    def __move_to_point(self, point):
        """
        Turns and translates to target.

        Args:
            point (list/array): target point in format [x (m), y (m)]
        """

        if point != [None, None] and self.robot_pose != [None, None, None]:
            target = point

            self.__turn_to_point(target, self.robot_pose)
            self.__translate_to_point(target, self.robot_pose)
        else:
            self.linear_vel = 0
            self.angular_vel = 0

        # publish velocity to cmd_vel
        vel = Twist()
        vel.linear.x = self.linear_vel
        vel.angular.z = self.angular_vel
        self.cmd_vel_publisher.publish(vel)

        # visualize path to target
        if self.target_point != [None, None]:
            self.__visualize_path_to_target(target=point)

    # ==================================================================================================================
    # VISUALIZATION
    # ==================================================================================================================

    def __visualize_path_to_target(self, target):
        marker = Marker()
        # Set the frame
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set the position of the point
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        start_point = Point(self.robot_pose[0], self.robot_pose[1], 0)
        end_point = Point(target[0], target[1], 0)

        marker.points.append(start_point)
        marker.points.append(end_point)

        # Set line properties
        marker.scale.x = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (transparency)

        self.path_segment_publisher.publish(marker)

    def __visualize_line_path(self, marker_list):
        marker = Marker()

        # Set the frame
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_paths"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Set the position of the point
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # set points to the list of marker points
        marker.points = marker_list

        # Set line properties
        marker.scale.x = 0.01
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # Alpha (transparency)

        self.path_publisher.publish(marker)

    # ==================================================================================================================
    # NODE RUNNING
    # ==================================================================================================================

    def run_node(self):
        rate = rospy.Rate(self.FREQUENCY)

        while not rospy.is_shutdown():
            # update difference in time
            self.pid_dt = rospy.Time.now().to_sec() - self.prev_pid_time
            self.prev_pid_time = rospy.Time.now().to_sec()  # update previous time

            if self.pid_dt == 0 or self.pid_dt is None: # ensure no div by 0 errors
                self.pid_dt = 1 / self.FREQUENCY  

            if self.path != [] and self.at_linear_target: #go to next target when at linear target
                # check that target point index will be within bounds and increment
                if self.target_point_index != len(self.path) - 1:
                    self.target_point_index += 1
                    self.target_point = self.path[self.target_point_index]

            self.__move_to_point(self.target_point)

            rate.sleep()


# ==================================================================================================================
# MAIN METHOD
# ==================================================================================================================

if __name__ == "__main__":
    point_to_point = PointToPoint()
    point_to_point.run_node()
