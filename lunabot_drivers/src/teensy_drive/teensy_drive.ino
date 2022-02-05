// include ROS and all messages
#include <deposition.h>
#include <actuation.h>
#include <drivetrain.h>

ros::NodeHandle nh;

void actuation_cb(const lunabot_msgs::Actuation& actuation) {
	actuation::run_actuation(actuation, nh);
}
void deposition_cb(const std_msgs::Float64& command) {
	deposition::run_deposition(command, nh);
}
void drivetrain_cb(const lunabot_msgs::Drivetrain& drive_msg) {
	drivetrain::run_drivetrain(drive_msg, nh);
}

ros::Subscriber<std_msgs::Float64> deposition_sub("/deposition", deposition_cb);
ros::Subscriber<lunabot_msgs::Actuation> actuation_sub("/actuation", actuation_cb);
ros::Subscriber<lunabot_msgs::Drivetrain> drivetrain_sub("/cmd_vel", drivetrain_cb);

void setup() {
  nh.initNode();
  nh.subscribe(drivetrain_sub);
  nh.subscribe(deposition_sub);
  nh.subscribe(actuation_sub);
	deposition::init();
	drivetrain::init();
	actuation::init();
}

void loop() {
  nh.spinOnce();
  delay(1);
}
