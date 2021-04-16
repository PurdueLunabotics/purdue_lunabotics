// include ROS and all messages
#include <ros.h>
#include <geometry_msgs/Twist.h>

// chassis motor counts
const int NUMMOTOR = 4;
const int NUMDIRECTION = 4;

// chassis motor indices
// corresponding entries are for the same motors
const int motorPins[NUMMOTOR] = {0, 4, 2, 6};           //order: front left, front right, back left, back right
const int directionPins[NUMDIRECTION] = {1, 5, 3, 7};

// create node handle
ros::NodeHandle nh;

// enums for determining motor pin index
#define FRONT 0
#define BACK 2
#define LEFT 0
#define RIGHT 1
#define MAX_VEL 1

// logic for running a motor
void moveMotor(int front_or_back, int left_or_right, int vel) {
  // index is found from adding the front/back and left/right enums
  int ind = front_or_back + left_or_right;
  int forward = (!left_or_right) ? HIGH : LOW; // left motors are opposite
  // if velocity is negative, reverse the motor
  
  if(vel > 0) {
    digitalWrite(directionPins[ind], forward);
  } else {
    digitalWrite(directionPins[ind], !forward);
  }

  // write the velocity to the pwm pin
  analogWrite(motorPins[ind], abs(vel));
}

void subscriberCallback(const geometry_msgs::Twist& cmd_vel) {
  /*

		Skid-steering configuration is implemented.
 
		Uses the twist to define the velocity components of the robot. the lin component is the heading forward and backward velocity
		and the angular z (ang) is the heading angle.
		Both lin and ang have range of [-1,1]. 
	*/

  double lin_v = cmd_vel.linear.x; // heading velocity
  double ang_v = cmd_vel.angular.z; // heading angle

  // calculate left and right chassis velocities
  int vel_l = map(lin_v - ang_v, -MAX_VEL, MAX_VEL, -255, 255); // Range from [-255,255]
  int vel_r = map(lin_v + ang_v, -MAX_VEL, MAX_VEL, -255, 255); // Range from [-255,255]
	vel_l = constrain(vel_l, -255, 255);
  vel_r = constrain(vel_r, -255, 255);

	nh.logerror("left_vel:");	
  nh.logerror(String(vel_l).c_str());
	nh.logerror("right_vel:");	
  nh.logerror(String(vel_r).c_str());
  // move each motor to the specified velocity
  moveMotor(FRONT, LEFT, vel_l);
  moveMotor(BACK, LEFT, vel_l);
  moveMotor(FRONT, RIGHT, vel_r);
  moveMotor(BACK, RIGHT, vel_r);

}

// create subscriber node
ros::Subscriber<geometry_msgs::Twist> comm_subscriber("cmd_vel", subscriberCallback);

void setup() {
  // set all pwm and direction pins to output
  for(int i = 0; i < NUMMOTOR; i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  for(int i = 0; i < NUMDIRECTION; i++) {
    pinMode(directionPins[i], OUTPUT);
  }

  // initialize the node and add subscriber
  nh.initNode();
  nh.subscribe(comm_subscriber);
}

void loop() {
  // spin up the node once in the loop
  nh.spinOnce();
  delay(20);
}
