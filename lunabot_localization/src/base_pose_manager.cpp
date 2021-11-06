// Publishes the base_link's odometry and tf transform
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>

class BasePoseManager {

  public:
    BasePoseManager(ros::NodeHandle* nodehandle) : nh_(*nodehandle) {
      initSubscribers();
      initPublishers();
    };

  private:
    ros::NodeHandle& nh_;
    void t265OdomCallback(ros::NodeHandle n);
    void initSubscribers();
    void initPublishers();
    void publishBaseTransform(ros::NodeHandle n);
};
void BasePoseManager::t265OdomCallback(ros::NodeHandle n) {
  ;
}

void BasePoseManager::publishBaseTransform(ros::NodeHandle n) {
  tf2::Quaternion rot;
  tf2::Vector3 trans;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_pose_manager");
  ros::NodeHandle n;

  ros::spin();
  return 0;
}