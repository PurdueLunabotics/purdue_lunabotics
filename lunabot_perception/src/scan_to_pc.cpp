#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

class ScanFilter {
public:
  ScanFilter();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

private:
  ros::NodeHandle node_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;

  ros::Publisher point_cloud_publisher_;
  ros::Subscriber scan_sub_;
};

ScanFilter::ScanFilter() {
  // tfListener_ = new tf::TransformListener();
  scan_sub_ =
      node_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &ScanFilter::scanCallback, this);
  point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>("/cloud", 100, false);
  tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

void ScanFilter::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  sensor_msgs::PointCloud2 cloud;
  try {
    projector_.transformLaserScanToPointCloud("laser", *scan, cloud, tfListener_);
  } catch (tf::TransformException &e) {
    ROS_ERROR("%s", e.what());
    return;
  }
  point_cloud_publisher_.publish(cloud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_filter");
  ScanFilter filter;
  ros::spin();

  return 0;
}