#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "sensors_tf_broadcaster");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.495, 0.172, 0.127)),
        ros::Time::now(),"base_link", "camera_left"));
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.495, -0.172, 0.127)),
        ros::Time::now(),"base_link", "camera_right"));
		broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.495, 0.150, 0.173)),
        ros::Time::now(),"base_link", "lidar"));
		r.sleep();
  }
}
