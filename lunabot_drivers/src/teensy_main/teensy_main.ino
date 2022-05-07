// include ROS and all messages
#include <actuation.h>
#include <drivetrain.h>
#include <excavation.h>
#include <deposition.h>

ros::NodeHandle nh;


void actuation_cb(const lunabot_msgs::Actuation& actuation) {
	actuation::run_actuation(actuation, &nh);
}
void excavation_cb(const std_msgs::Float64& speed) {
	excavation::run_excavation(speed, &nh);
}
void deposition_cb(const std_msgs::Int8& command) {
	deposition::run_deposition(command, &nh);
}
void drivetrain_cb(const lunabot_msgs::Drivetrain& drive_msg) {
	drivetrain::run_drivetrain(drive_msg, &nh);
}

ros::Subscriber<lunabot_msgs::Actuation> actuation_sub("/actuation", actuation_cb);
ros::Subscriber<std_msgs::Float64> excavation_sub("/excavation", excavation_cb);
ros::Subscriber<std_msgs::Int8> deposition_sub("/deposition", deposition_cb);
ros::Subscriber<lunabot_msgs::Drivetrain> drivetrain_sub("/cmd_vel", drivetrain_cb);

void setup() {
	nh.initNode();
	nh.subscribe(drivetrain_sub);
	nh.subscribe(deposition_sub);
	nh.subscribe(actuation_sub);
	nh.subscribe(excavation_sub);
	deposition::init();
	drivetrain::init();
	actuation::init();
	excavation::init();

}

void loop() {
	nh.spinOnce();
	actuation::stepper_step();
	delay(10);
}
