#include <robot.hpp>

// sensor wire documentation 2022-2023:
// https://docs.google.com/spreadsheets/d/1eX79YtawJqBA8VePFFtKJT6RR21gvK4qH1DKJX_vJE0/edit#gid=0

// ADCs
namespace actuation {

StepperMotor act_right_mtr(0x01); // TODO RJN - these should init in main...
StepperMotor act_left_mtr(0x02);

void update(float &act_right_curr) {
  act_right_curr = act_right_mtr.read_current();
}

void cb(int8_t lin_act_volt) { // TODO RJN - convert from volts to rpm
  act_left_mtr.move_at_speed(lin_act_volt);
  act_right_mtr.move_at_speed(lin_act_volt);
}

} // namespace actuation

namespace drivetrain {
StepperMotor left_drive_mtr(0x03);
StepperMotor right_drive_mtr(0x04);

void update(float &left_curr, float &right_curr, float &left_angle, float &right_angle) {
  left_curr = left_drive_mtr.read_current();
  right_curr = right_drive_mtr.read_current();
  left_angle = left_drive_mtr.read_motor_position_radians();
  right_angle = -right_drive_mtr.read_motor_position_radians();
}

float update_curr_left() {
  return left_drive_mtr.read_current();
}

float update_curr_right() {
  return right_drive_mtr.read_current();
}

void cb(int8_t left_drive_volt, int8_t right_drive_volt) { // TODO RJN - convert from volts to rpm
  // Tank drive steering
  left_drive_mtr.move_at_speed(left_drive_volt);
  right_drive_mtr.move_at_speed(right_drive_volt);
}

} // namespace drivetrain

namespace uwb {
void update(float &d0, float &d1, float &d2) {
  d0 = M5Stack_UWB_Trncvr::read_uwb(0);
  d1 = M5Stack_UWB_Trncvr::read_uwb(1);
  d2 = M5Stack_UWB_Trncvr::read_uwb(2);
}
} // namespace uwb

namespace excavation {
StepperMotor exc_mtr(0x05);

void update(float &exc_curr, float &exc_angle) {
  exc_curr = exc_mtr.read_current();
  exc_angle = -exc_mtr.read_motor_position_radians();
}

float update_curr() {
  return exc_mtr.read_current();
}

void cb(int8_t speed) {
  exc_mtr.move_at_speed(speed);
}
} // namespace excavation

namespace deposition {
StepperMotor dep_mtr(0x06);

void update(float &dep_curr) {
  dep_curr = dep_mtr.read_current();
}

float update_curr() {
  return dep_mtr.read_current();
}

void cb(int8_t volt) {
  dep_mtr.move_at_speed(-volt);
}

} // namespace deposition
