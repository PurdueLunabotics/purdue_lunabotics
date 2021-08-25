#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include <math.h>

using namespace std;

void sendStaticTransform(double translation[] ,double rotation[], string child, string parent) {
	static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = parent;
  static_transformStamped.child_frame_id = child;
  static_transformStamped.transform.translation.x = translation[0];
  static_transformStamped.transform.translation.y = translation[1];
  static_transformStamped.transform.translation.z = translation[2];
  
	tf2::Quaternion quat;
  quat.setRPY(rotation[0], rotation[1], rotation[2]);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  
	static_broadcaster.sendTransform(static_transformStamped);
}

void sendDynamicTransform(double translation[], double rotation[], string child, string parent)
{
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = parent;
  transformStamped.child_frame_id = child;
  transformStamped.transform.translation.x = translation[0];
  transformStamped.transform.translation.y = translation[1];
  transformStamped.transform.translation.z = translation[2];
  
	tf2::Quaternion quat;
  quat.setRPY(rotation[0], rotation[1], rotation[2]);
  transformStamped.transform.rotation.x = quat.x();
  transformStamped.transform.rotation.y = quat.y();
  transformStamped.transform.rotation.z = quat.z();
  transformStamped.transform.rotation.w = quat.w();
  
	br.sendTransform(transformStamped);
}

void lidar_angle_cb(const std_msgs::Float64& angle){

	double lidar_trans[3] = {0.495, 0.173, 0.150};
	double lidar_rot[3] = {0,angle.data,0};

	sendDynamicTransform(lidar_trans, lidar_rot, "laser", "base_link");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "sensors_tf_broadcaster");
  ros::NodeHandle n;
	ros::Subscriber lidar_angle_sub = n.subscribe("lidar_angle", 1000, lidar_angle_cb);	
	double camera_left_trans[3] = {-0.495,0.127,0.172};
	double camera_right_trans[3] = {-0.495, -0.127, 0.172};
	double camera_rot[3] = {-M_PI/2,0,M_PI/2};
	
	sendStaticTransform(camera_left_trans, camera_rot, "unzano", "base_link");
	sendStaticTransform(camera_right_trans, camera_rot, "tessar", "base_link");
	
	ros::spin();

	return 0;
}
