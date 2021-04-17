// include ROS and all messages
#include <excavation.h>

ros::NodeHandle nh;

void excavation_cb(const std_msgs::Float64& speed) {
	excavation::run_excavation(speed, nh);
}

ros::Subscriber<std_msgs::Float64> excavation_sub("excavation", excavation_cb);

void setup() {
  nh.initNode();
  nh.subscribe(excavation_sub);
	excavation::init();
}

void loop() {
  nh.spinOnce();
  delay(20);
}
