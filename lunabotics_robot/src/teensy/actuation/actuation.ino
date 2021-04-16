#include <ros.h>
#include <std_msgs/Float64.h>

// ex-dep motor counts
static const uint8_t MOTOR_CNT_ = 3;

// ex-dep motor indices
// corresponding entries are for the same motor
const int drive_pins_[MOTOR_CNT_] = { 10, 14, 18 };     //order: lead screw actuation, rotation actuation right, rotation actuation left 
const int direction_pins_[MOTOR_CNT_] = { 11, 15, 19 };

// create node handle
ros::NodeHandle nh;

// TODO: Implement move angle
void _move_angle(int speed) {
	// (HIGH (CW and moves DOWN), LOW (CCW and moves UP))
	int direction = (speed > 0) ? LOW : HIGH;
	digitalWrite(direction_pins_[1], direction);
	digitalWrite(direction_pins_[2], direction);

  analogWrite(drive_pins_[1], abs(speed));
  analogWrite(drive_pins_[2], abs(speed));
}

void _move_lead_screw(int speed) {
	int direction = (speed > 0) ? LOW : HIGH;
	digitalWrite(direction_pins_[0], direction);

  analogWrite(drive_pins_[0], abs(speed));
}

void _actuation_cb(const std_msgs::Float64& actuate) {
  int vel = map(actuate.data, -1, 1, -255, 255); // Range from [-255,255]
  vel = constrain(vel, -255, 255);

  nh.logerror("Actuation:");  
  nh.logerror(String(vel).c_str());
  
  _move_angle(vel);
}

ros::Subscriber<std_msgs::Float64> actuation_sub("actuation", &_actuation_cb);

void setup() {
  nh.initNode();
 	nh.subscribe(actuation_sub);

  for(int i = 0; i < MOTOR_CNT_; i++) {
    pinMode(drive_pins_[i], OUTPUT);
  }

  for(int i = 0; i < MOTOR_CNT_; i++) {
    pinMode(direction_pins_[i], OUTPUT);
  }  
}

void loop() {
	nh.spinOnce();
	delay(20);
  // spin up the node once in the loop
}
