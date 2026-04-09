#include "StepperLib.hpp"
#include "interfaces.hpp"
#include <robot.hpp>

// sensor wire documentation 2022-2023:
// https://docs.google.com/spreadsheets/d/1eX79YtawJqBA8VePFFtKJT6RR21gvK4qH1DKJX_vJE0/edit#gid=0


namespace drivetrain {
StepperMotor left_drive_mtr(LEFT_DRIVE_MOTOR_ID, ISV2);
StepperMotor right_drive_mtr(RIGHT_DRIVE_MOTOR_ID, ISV2);

void begin() {
  left_drive_mtr.begin();
  right_drive_mtr.begin();
}

void update(float &left_curr, float &right_curr, float &left_torque, float &right_torque, float &left_vel, float &right_vel) {
  left_curr = -1 * left_drive_mtr.read_current();
  right_curr = right_drive_mtr.read_current();
  left_torque = -1 * left_drive_mtr.read_torque(); // -100; // left_drive_mtr.read_motor_position_radians();
  right_torque = right_drive_mtr.read_torque(); // ; // right_drive_mtr.read_motor_position_radians();
  left_vel = left_drive_mtr.read_velocity();
  right_vel = right_drive_mtr.read_velocity();
}

float update_curr_left() {
  return left_drive_mtr.read_current();
}

float update_curr_right() {
  return right_drive_mtr.read_current();
}

void cb(int32_t left_drive_rpm, int32_t right_drive_rpm, bool should_reset) {
  // Tank drive steering
  if (should_reset) {
    left_drive_mtr.clear_errors();
    right_drive_mtr.clear_errors();
  } else {
    left_drive_mtr.move_at_speed(-1 * left_drive_rpm);
    right_drive_mtr.move_at_speed(right_drive_rpm);
  }
}

} // namespace drivetrain

namespace LEDs {
  void cb(int32_t color, uint8_t counter) {
    Led_Strip::set_color(color,counter);
  }
}

namespace deposition {
StepperMotor dep_mtr(DEP_MOTOR_ID, BLD305S);

void begin() {
  dep_mtr.begin();
}

void update(float &dep_curr) {
  dep_curr = dep_mtr.read_current();
}

float update_curr() {
  return dep_mtr.read_current();
}

void cb(int32_t speed_rpm, bool should_reset) {
  if (should_reset) {
    dep_mtr.clear_errors();
  } else {
    dep_mtr.move_at_speed(-speed_rpm);
  }
}

} // namespace deposition
