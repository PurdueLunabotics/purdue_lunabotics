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
Sabertooth_MotorCtrl lead_screw_mtr{&MC1, STMotor::M1};

constexpr uint8_t ACT_RIGHT_CURR_ADC = 1;
constexpr uint8_t ACT_RIGHT_CURR_MUX = 1; // U6 curr_sense_board

constexpr uint8_t ACT_LEFT_CURR_ADC = 0;
constexpr uint8_t ACT_LEFT_CURR_MUX = 3; // U4 curr_sense_board

constexpr uint8_t LEAD_SCREW_CURR_ADC = 1;
constexpr uint8_t LEAD_SCREW_CURR_MUX = 2; // U7 curr_sense_board

constexpr uint8_t LEAD_SCREW_ENC_MUX = 4;
constexpr uint8_t ACT_ENC_MUX = 2;

void update(int32_t &act_right_curr, int32_t &lead_screw_curr, float &act_angle,
            float &lead_screw_angle) {
  act_right_curr = ACS711_Current_Bus::read(ACT_RIGHT_CURR_ADC, ACT_RIGHT_CURR_MUX);
  lead_screw_curr = ACS711_Current_Bus::read(LEAD_SCREW_CURR_ADC, LEAD_SCREW_CURR_MUX);
  lead_screw_angle = VLH35_Angle_Bus::read_enc(LEAD_SCREW_ENC_MUX);
  act_angle = VLH35_Angle_Bus::read_enc(ACT_ENC_MUX);
}

void cb(int8_t lead_screw_volt, int8_t lin_act_volt) {
  act_left_mtr.write(-lin_act_volt);
  act_right_mtr.write(lin_act_volt);
  lead_screw_mtr.write(lead_screw_volt);
}

} // namespace actuation

namespace drivetrain {
Sabertooth_MotorCtrl left_drive_mtr{&MC3, STMotor::M1};
Sabertooth_MotorCtrl right_drive_mtr{&MC3, STMotor::M2};

constexpr uint8_t LEFT_CURR_ADC = 0;
constexpr uint8_t LEFT_CURR_MUX = 2; // U3 curr_sense_board

constexpr uint8_t RIGHT_CURR_ADC = 0;
constexpr uint8_t RIGHT_CURR_MUX = 0; // U1 curr_sense_board

constexpr uint8_t DRIVE_RIGHT_MUX = 0;
constexpr uint8_t DRIVE_LEFT_MUX = 3;

void update(int32_t &left_curr, int32_t &right_curr, float &left_angle, float &right_angle) {
  left_curr = ACS711_Current_Bus::read(LEFT_CURR_ADC, LEFT_CURR_MUX);

  right_curr = ACS711_Current_Bus::read(RIGHT_CURR_ADC, RIGHT_CURR_MUX);

  left_angle = VLH35_Angle_Bus::read_enc(DRIVE_LEFT_MUX);
  right_angle = VLH35_Angle_Bus::read_enc(DRIVE_RIGHT_MUX);
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

void update(int32_t &exc_curr) { exc_curr = ACS711_Current_Bus::read(EXC_CURR_ADC, EXC_CURR_MUX); }

void cb(int8_t speed) { exc_mtr.write(speed); }
} // namespace excavation

namespace deposition {
Sabertooth_MotorCtrl dep_mtr{&MC1, STMotor::M2};
constexpr uint8_t DEP_CURR_ADC = 0;
constexpr uint8_t DEP_CURR_MUX = 1; // U2 curr_sense_board

constexpr uint8_t DEP_ENC_MUX = 1;

void update(int32_t &dep_curr, float &dep_angle) {
  dep_curr = ACS711_Current_Bus::read(DEP_CURR_ADC, DEP_CURR_MUX);
  dep_angle = VLH35_Angle_Bus::read_enc(DEP_ENC_MUX);
}

void cb(int8_t volt) { dep_mtr.write(volt); }

} // namespace deposition

namespace load_cell {
void update(float &d0, float &d1) {
  d0 = HX711_Load_Cell::read_weight(0);
  d1 = HX711_Load_Cell::read_weight(1);
}
} // namespace load_cell
