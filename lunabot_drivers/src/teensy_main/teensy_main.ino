// include ROS and all messages
#include <actuation.h>
#include <drivetrain.h>
#include <excavation.h>
#include <deposition.h>
#include <lunabot_msgs/ExcavationState.h>
#include <lunabot_msgs/LunabotState.h>

ros::NodeHandle nh;

void actuation_cb(const lunabot_msgs::Actuation &actuation)
{
	actuation::run_actuation(actuation, &nh);
}
void excavation_cb(const std_msgs::Float32 &speed)
{
	excavation::run_excavation(speed, &nh);
}
void deposition_cb(const std_msgs::Int8 &command)
{
	deposition::run_deposition(command, &nh);
}
void drivetrain_cb(const lunabot_msgs::Drivetrain &drive_msg)
{
	drivetrain::run_drivetrain(drive_msg, &nh);
}

// 32 bytes in/out total
ros::Subscriber<lunabot_msgs::Actuation> actuation_sub("/actuation", actuation_cb);
ros::Subscriber<std_msgs::Float32> excavation_sub("/excavation", excavation_cb);
ros::Subscriber<std_msgs::Int8> deposition_sub("/deposition", deposition_cb);
ros::Subscriber<lunabot_msgs::Drivetrain> drivetrain_sub("/cmd_vel", drivetrain_cb);

lunabot_msgs::LunabotState robot_state;
ros::Publisher robot_state_pub("/robot_state", &robot_state);

void publish_lunabot_state()
{
	robot_state.dep_state = deposition::dep_hall.state;
	robot_state.lin_act_state = actuation::lin_act_hall.state;
	robot_state.lead_screw_state =  actuation::lead_screw_hall.state;
	robot_state.exc_state.bin_state =  excavation::exc_feedback.bin_state;
	robot_state.exc_state.exc_state =  excavation::exc_feedback.exc_state;
	robot_state.exc_state.current_draw = excavation::exc_current.current_value;
	robot_state.exc_state.bin_weight = excavation::exc_feedback.current_wt;
	robot_state_pub.publish(&robot_state);
}

void setup()
{
	nh.initNode();
	nh.subscribe(drivetrain_sub);
	nh.subscribe(deposition_sub);
	nh.subscribe(actuation_sub);
	nh.subscribe(excavation_sub);
	nh.advertise(robot_state_pub);
	deposition::init();
	drivetrain::init();
	actuation::init();
	excavation::init();
}

void loop()
{
	nh.spinOnce();
	publish_lunabot_state();
	actuation::stepper_step();
	delay(50);
}
