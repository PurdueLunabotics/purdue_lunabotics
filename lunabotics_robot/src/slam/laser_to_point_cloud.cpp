#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n, std::string _laser_frame_id = "laser", std::string _scan_name = "scan") : 
    n_(n),
    laser_sub_(n_, _scan_name, 10),
    laser_notifier_(laser_sub_,listener_, _laser_frame_id, 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1,_laser_frame_id));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/point_cloud",1);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in, std::string _laser_frame_id)
  {
    sensor_msgs::PointCloud cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          _laser_frame_id,*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    
    // Do something with cloud.

    scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}
