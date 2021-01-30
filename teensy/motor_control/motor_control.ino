// include ROS and all messages
#include <ros.h>
#include <geometry_msgs/Twist.h>

// chassis motor counts
const int NUMMOTOR = 4;
const int NUMDIRECTION = 4;

// chassis motor indices
// corresponding entries are for the same motor
const int motorPins[NUMMOTOR] = {4, 8, 2, 6};           //order: front left, front right, back left, back right
const int directionPins[NUMDIRECTION] = {5, 9, 3, 7};

// create node handle
ros::NodeHandle node_handle;

// enums for determining motor pin index
#define FRONT 0
#define BACK 2
#define LEFT 0
#define RIGHT 1

// logic for running a motor
void moveMotor(int front_or_back, int left_or_right, int vel) {
  // index is found from adding the front/back and left/right enums
  int ind = front_or_back + left_or_right;

  // if velocity is negative, reverse the motor
  if(vel > 0) {
    digitalWrite(directionPins[ind], HIGH);
  } else {
    digitalWrite(directionPins[ind], LOW);
  }

  // write the velocity to the pwm pin
  analogWrite(motorPins[ind], abs(vel));
}

void subscriberCallback(const geometry_msgs::Twist& command) {
  // this code uses only the linear vector of the twist to define
  // the velocity of the robot. the y component is forward and
  // backward motion and the x component is left and right turning.
  // both y and x have magnitude 1 or less. positive x is turning to
  // the right

  // more math is needed to refine the motion

  // extract velocity components from twist
  double lin = command.linear.y;
  double ang = command.linear.x;

  // calculate left and right chassis velocities
  int vel_l = lin * 127 + ang * 127;
  int vel_r = lin * 127 - ang * 127;

  // move each motor to the specified velocity
  moveMotor(FRONT, LEFT, vel_l);
  moveMotor(BACK, LEFT, vel_l);
  moveMotor(FRONT, RIGHT, vel_r);
  moveMotor(BACK, RIGHT, vel_r);

}

// create subscriber node
ros::Subscriber<geometry_msgs::Twist> comm_subscriber("motor_command", subscriberCallback);

void setup() {
  // set all pwm and direction pins to output
  for(int i = 0; i < NUMMOTOR; i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  for(int i = 0; i < NUMDIRECTION; i++) {
    pinMode(directionPins[i], OUTPUT);
  }

  // initialize the node and add subscriber
  node_handle.initNode();
  node_handle.subscribe(comm_subscriber);
}

void loop() {
  // spin up the node once in the loop
  node_handle.spinOnce();
  delay(100);
}
