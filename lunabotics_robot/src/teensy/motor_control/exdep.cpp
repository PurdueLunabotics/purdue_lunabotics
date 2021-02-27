#include <ros.h>
#include <std_msgs/Byte.h>

namespace exdep {

// ex-dep motor counts
const int NUM_EXDEP_MOTOR = 5;
const int NUM_EXDEP_DIRECTION = 5;

// ex-dep motor indices
// corresponding entries are for the same motor
const int exdepMotorPins[NUM_EXDEP_MOTOR] = {1, 3, 5, 7, 9};           //order: excavation x 2, actuation x 2, deposition x 1
const int exdepDirectionPins[NUM_EXDEP_DIRECTION] = {2, 4, 6, 8, 10};

void init() {
  for(int i = 0; i < NUM_EXDEP_MOTOR; i++) {
    pinMode(exdepMotorPins[i], OUTPUT);
  }

  for(int i = 0; i < NUM_EXDEP_DIRECTION; i++) {
    pinMode(exdepDirectionPins[i], OUTPUT);
  }
}

// enums for determining exdep motor pin index
#define EXCAVATION 0
#define ACTUATION 2
#define DEPOSITION 4

// logic for running a motor
void moveExdepMotor(int ind, int command) {

  // if velocity is negative, reverse the motor
  if(command == 1) {
    digitalWrite(exdepDirectionPins[ind], HIGH);
    if(ind != 4) {
      digitalWrite(exdepDirectionPins[ind + 1], HIGH);
    }
  } else {
    digitalWrite(exdepDirectionPins[ind], LOW);
    if(ind != 4) {
      digitalWrite(exdepDirectionPins[ind + 1], LOW);
    }
  }

  // write the velocity to the pwm pin
  if(command != 0) {
    analogWrite(exdepMotorPins[ind], 255);
    if(ind != 4) {
      analogWrite(exdepMotorPins[ind + 1], 255);
    }
  } else {
    analogWrite(exdepMotorPins[ind], 0);
    if(ind != 4) {
      analogWrite(exdepMotorPins[ind + 1], 0);
    }
  }
}

void actuateExdep(const std_msgs::Byte& command) {
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

}
