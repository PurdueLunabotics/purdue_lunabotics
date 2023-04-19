#include <robot.hpp>

// sensor wire documentation 2022-2023:
// https://docs.google.com/spreadsheets/d/1eX79YtawJqBA8VePFFtKJT6RR21gvK4qH1DKJX_vJE0/edit#gid=0

// MCs
Sabertooth MC1(128, ST_SERIAL); // top
Sabertooth MC2(129, ST_SERIAL); // middle
Sabertooth MC3(130, ST_SERIAL); // bottom
Sabertooth MC4(131, ST_SERIAL); // high-current

// ADCs

ADS1115_lite adc0(ADS1115_ADDRESS_ADDR_SCL);
ADS1115_lite adc1(ADS1115_ADDRESS_ADDR_SDA);

namespace actuation {

STMotorInterface act_right{&MC2, STMotor::M1};
STMotorInterface act_left{&MC2, STMotor::M2};
STMotorInterface lead_screw_motor{&MC1, STMotor::M1};

CurrentSensor act_right_curr{&adc1, ADSChannel::A1_ch};  // U6 curr_sense_board
CurrentSensor act_left_curr{&adc0, ADSChannel::A3_ch};   // U4 curr_sense_board
CurrentSensor lead_screw_curr{&adc1, ADSChannel::A2_ch}; // U7 curr_sense_board

void update(int16_t *act_right, int16_t *lead_screw) {
    *act_right = act_right_curr.read();
    *lead_screw = lead_screw_curr.read();
}

void loop_once() {
    act_right_curr.loop();
    // act_left_curr.loop();
    lead_screw_curr.loop();
}

void cb(int8_t lead_screw, int8_t lin_act) {
  act_left.write(-lin_act);
  act_right.write(lin_act);
  lead_screw_motor.write(lead_screw);
}

} // namespace actuation

namespace drivetrain {
STMotorInterface left_drive{&MC3, STMotor::M1};
STMotorInterface right_drive{&MC3, STMotor::M2};

CurrentSensor left_drive_curr{&adc0, ADSChannel::A2_ch};  // U3 curr_sense_board
CurrentSensor right_drive_curr{&adc0, ADSChannel::A0_ch}; // U1 curr_sense_board

void update(int16_t *left, int16_t *right) {
    *left = left_drive_curr.read();
    *right = right_drive_curr.read();
}

void loop_once() {
    left_drive_curr.loop();
    right_drive_curr.loop();
}

void cb(int8_t left, int8_t right) {
  // Tank drive steering
  left_drive.write(-left);
  right_drive.write(-right);
}

} // namespace drivetrain

namespace excavation {
STMotorInterface exc_motor{&MC4, STMotor::M1};

CurrentSensor exc_curr{&adc1, ADSChannel::A0_ch}; // U5 curr_sense_board

void update(int16_t *exc) { *exc = exc_curr.read(); }

void loop_once() { exc_curr.loop(); }

void cb(int8_t speed) { exc_motor.write(speed); }
} // namespace excavation

namespace deposition {
STMotorInterface dep_motor{&MC1, STMotor::M2};

CurrentSensor dep_curr{&adc0, ADSChannel::A1_ch}; // U2 curr_sense_board

void update(int16_t *dep) { *dep = dep_curr.read(); }

void loop_once() { dep_curr.loop(); }
void cb(int8_t speed) { dep_motor.write(speed); }

} // namespace deposition
