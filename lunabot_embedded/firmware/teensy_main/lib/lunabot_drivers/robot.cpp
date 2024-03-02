#include <robot.hpp>

// sensor wire documentation 2022-2023:
// https://docs.google.com/spreadsheets/d/1eX79YtawJqBA8VePFFtKJT6RR21gvK4qH1DKJX_vJE0/edit#gid=0

// MCs
Sabertooth MC1(128, ST_SERIAL); // top
Sabertooth MC2(129, ST_SERIAL); // middle
Sabertooth MC3(130, ST_SERIAL); // bottom
Sabertooth MC4(131, ST_SERIAL); // high-current

// ADCs
namespace actuation {

Sabertooth_MotorCtrl act_right_mtr{&MC2, STMotor::M1};
Sabertooth_MotorCtrl act_left_mtr{&MC2, STMotor::M2};

constexpr uint8_t ACT_RIGHT_CURR_ADC = 1;
constexpr uint8_t ACT_RIGHT_CURR_MUX = 1; // U6 curr_sense_board

constexpr uint8_t ACT_LEFT_CURR_ADC = 0;
constexpr uint8_t ACT_LEFT_CURR_MUX = 3; // U4 curr_sense_board

void update(int32_t &act_right_curr) {
  act_right_curr = ACS711_Current_Bus::read(ACT_RIGHT_CURR_ADC, ACT_RIGHT_CURR_MUX);
}

void cb(int8_t lin_act_volt) {
  act_left_mtr.write(-lin_act_volt);
  act_right_mtr.write(lin_act_volt);
}

} // namespace actuation

namespace drivetrain {
Sabertooth_MotorCtrl left_drive_mtr{&MC3, STMotor::M1};
Sabertooth_MotorCtrl right_drive_mtr{&MC3, STMotor::M2};

constexpr uint8_t LEFT_CURR_ADC = 0;
constexpr uint8_t LEFT_CURR_MUX = 2; // U3 curr_sense_board

constexpr uint8_t RIGHT_CURR_ADC = 0;
constexpr uint8_t RIGHT_CURR_MUX = 0; // U1 curr_sense_board

constexpr uint8_t LEFT_ENC_ID = 0;
constexpr uint8_t RIGHT_ENC_ID = 0;

void update(int32_t &left_curr, int32_t &right_curr, float &left_angle, float &right_angle) {
  left_curr = ACS711_Current_Bus::read(LEFT_CURR_ADC, LEFT_CURR_MUX);

  right_curr = ACS711_Current_Bus::read(RIGHT_CURR_ADC, RIGHT_CURR_MUX);

  left_angle = AMT13_Angle_Bus::read_enc(LEFT_ENC_ID);
  right_angle = AMT13_Angle_Bus::read_enc(RIGHT_ENC_ID);
}

void cb(int8_t left_drive_volt, int8_t right_drive_volt) {
  // Tank drive steering
  left_drive_mtr.write(left_drive_volt);
  right_drive_mtr.write(right_drive_volt);
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
Sabertooth_MotorCtrl exc_mtr{&MC4, STMotor::M1};

constexpr uint8_t EXC_CURR_ADC = 1;
constexpr uint8_t EXC_CURR_MUX = 0; // U5 curr_sense_board

constexpr uint8_t EXC_ENC_ID = 2;

void update(int32_t &exc_curr, float &exc_angle) { 
  exc_curr = ACS711_Current_Bus::read(EXC_CURR_ADC, EXC_CURR_MUX); 
  exc_angle = AMT13_Angle_Bus::read_enc(EXC_ENC_ID);
}

void cb(int8_t speed) { exc_mtr.write(speed); }
} // namespace excavation

namespace deposition {
Sabertooth_MotorCtrl dep_mtr{&MC1, STMotor::M2};
constexpr uint8_t DEP_CURR_ADC = 0;
constexpr uint8_t DEP_CURR_MUX = 1; // U2 curr_sense_board

void update(int32_t &dep_curr) {
  dep_curr = ACS711_Current_Bus::read(DEP_CURR_ADC, DEP_CURR_MUX);
}

void cb(int8_t volt) { dep_mtr.write(volt); }

} // namespace deposition