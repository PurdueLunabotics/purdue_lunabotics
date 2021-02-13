// include ROS and all messages
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Byte.h>

// chassis motor counts
const int NUM_CHASSIS_MOTOR = 4;
const int NUM_CHASSIS_DIRECTION = 4;

// chassis motor indices
// corresponding entries are for the same motor
const int driveMotorPins[NUM_CHASSIS_MOTOR] = {4, 8, 2, 6};           //order: front left, front right, back left, back right
const int driveDirectionPins[NUM_CHASSIS_DIRECTION] = {5, 9, 3, 7};

// ex-dep motor counts
const int NUM_EXDEP_MOTOR = 3;
const int NUM_EXDEP_DIRECTION = 3;

// ex-dep motor indices
// corresponding entries are for the same motor
const int exdepMotorPins[NUM_EXDEP_MOTOR] = {1, 3, 5};           //order: excavation, actuation, deposition
const int exdepDirectionPins[NUM_EXDEP_DIRECTION] = {2, 4, 6};

// create node handle
ros::NodeHandle node_handle;

// enums for determining chassis motor pin index
#define FRONT 0
#define BACK 2
#define LEFT 0
#define RIGHT 1

// logic for running a motor
void moveChassisMotor(int front_or_back, int left_or_right, int vel) {
  // index is found from adding the front/back and left/right enums
  int ind = front_or_back + left_or_right;

  // if velocity is negative, reverse the motor
  if(vel > 0) {
    digitalWrite(driveDirectionPins[ind], HIGH);
  } else {
    digitalWrite(driveDirectionPins[ind], LOW);
  }

  // write the velocity to the pwm pin
  analogWrite(driveMotorPins[ind], abs(vel));
}

// enums for determining exdep motor pin index
#define EXCAVATION 0
#define ACTUATION 1
#define DEPOSITION 2

// logic for running a motor
void moveExdepMotor(int ind, int command) {

  // if velocity is negative, reverse the motor
  if(command == 1) {
    digitalWrite(exdepDirectionPins[ind], HIGH);
  } else {
    digitalWrite(exdepDirectionPins[ind], LOW);
  }

  // write the velocity to the pwm pin
  if(command != 0) {
    analogWrite(exdepMotorPins[ind], 255);
  } else {
    analogWrite(exdepMotorPins[ind], 0);
  }
}

void driveCallback(const geometry_msgs::Twist& command) {
  /*

		Skid-steering configuration is implemented.
 
		Uses the twist to define the velocity components of the robot. the lin component is the heading forward and backward velocity
		and the angular z (ang) is the heading angle.
		Both lin and ang have range of [-1,1]. 
	*/

  double lin = command.linear.x; // heading velocity
  double ang = command.angular.z; // heading angle

  // calculate left and right chassis velocities
  int vel_l = constrain(lin * 128 - ang * 128, -255, 255); // Range from [-255,255]
  int vel_r = constrain(lin * 128 + ang * 128, -255, 255); // Range from [-255,255]

  // move each motor to the specified velocity
  moveChassisMotor(FRONT, LEFT, vel_l);
  moveChassisMotor(BACK, LEFT, vel_l);
  moveChassisMotor(FRONT, RIGHT, vel_r);
  moveChassisMotor(BACK, RIGHT, vel_r);

}

void exdepCallback(const std_msgs::Byte& command) {
  // byte layout
  // XXEEAADD
  // 0 = dont move subsystem
  // 1 = move subsystem forwards
  // 2 = move subsystem backwards

  // ex: move excavation forwards, dont move actuation, move deposition backwards
  // command: 00 01 00 10 = 18
  
  byte data = command.data;

  int deposition = data & 0x3;
  data >>= 2;
  int actuation = data & 0x3;
  data >>= 2;
  int excavation = data & 0x3;

  moveExdepMotor(EXCAVATION, excavation);
  moveExdepMotor(ACTUATION, actuation);
  moveExdepMotor(DEPOSITION, deposition);
}

// create subscriber nodes
ros::Subscriber<geometry_msgs::Twist> comm_subscriber("motor_command", driveCallback);
ros::Subscriber<std_msgs::Byte> exdep_subscriber("ex_dep_control", exdepCallback);

void setup() {
  // set all pwm and direction pins to output
  for(int i = 0; i < NUM_CHASSIS_MOTOR; i++) {
    pinMode(driveMotorPins[i], OUTPUT);
  }

  for(int i = 0; i < NUM_CHASSIS_DIRECTION; i++) {
    pinMode(driveDirectionPins[i], OUTPUT);
  }

  for(int i = 0; i < NUM_EXDEP_MOTOR; i++) {
    pinMode(exdepMotorPins[i], OUTPUT);
  }

  for(int i = 0; i < NUM_EXDEP_DIRECTION; i++) {
    pinMode(exdepDirectionPins[i], OUTPUT);
  }

  // initialize the node and add subscriber
  node_handle.initNode();
  node_handle.subscribe(comm_subscriber);
  node_handle.subscribe(exdep_subscriber);
}

void loop() {
  // spin up the node once in the loop
  node_handle.spinOnce();
  delay(20);
}
