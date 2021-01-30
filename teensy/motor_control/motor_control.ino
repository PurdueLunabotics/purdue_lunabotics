#include <ros.h>
#include <geometry_msgs/Twist.h>

const int NUMMOTOR = 4;
const int NUMDIRECTION = 4;

const int motorPins[NUMMOTOR] = {4, 8, 2, 6};           //fl fr bl br
const int directionPins[NUMDIRECTION] = {5, 9, 3, 7};

ros::NodeHandle node_handle;

void subscriberCallback(const geometry_msgs::Twist& command) {
  /*

		Skid-steering configuration is implemented.
 
		Uses the twist to define the velocity components of the robot. the lin component is the heading forward and backward velocity
		and the angular z (ang) is the heading angle.
		Both lin and ang have magnitude [-1,1]. 
	*/

  double lin_v = command.linear.x; // heading velocity
  double ang = command.angular.z; // heading angle

  int vel_l = constrain(lin * 128 - ang * 128, -255, 255); // Range from [-255,255]
  int vel_r = constrain(lin * 128 + ang * 128, -255, 255); // Range from [-255,255]

  if(vel_l > 0) {
    digitalWrite(directionPins[0], HIGH);
    digitalWrite(directionPins[2], HIGH);
  } else {
    digitalWrite(directionPins[0], LOW);
    digitalWrite(directionPins[2], LOW);
  }

  analogWrite(motorPins[0], abs(vel_l));
  analogWrite(motorPins[2], abs(vel_l));

  if(vel_r > 0) {
    digitalWrite(directionPins[1], HIGH);
    digitalWrite(directionPins[3], HIGH);
  } else {
    digitalWrite(directionPins[1], LOW);
    digitalWrite(directionPins[3], LOW);
  }

  analogWrite(motorPins[1], abs(vel_r));
  analogWrite(motorPins[3], abs(vel_r));

}

ros::Subscriber<geometry_msgs::Twist> comm_subscriber("motor_command", subscriberCallback);

void setup()
{
  for(int i = 0; i < NUMMOTOR; i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  for(int i = 0; i < NUMDIRECTION; i++) {
    pinMode(directionPins[i], OUTPUT);
  }
  
  node_handle.initNode();
  node_handle.subscribe(comm_subscriber);
}

void loop()
{ 
  node_handle.spinOnce();
  delay(20);
}
