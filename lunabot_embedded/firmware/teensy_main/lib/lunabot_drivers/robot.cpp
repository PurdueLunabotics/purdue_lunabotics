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

CurrentSensor act_right_curr_sense{&adc1,
                                   ADSChannel::A1_ch}; // U6 curr_sense_board
CurrentSensor act_left_curr_sense{&adc0,
                                  ADSChannel::A3_ch}; // U4 curr_sense_board
CurrentSensor lead_screw_curr_sense{&adc1,
                                    ADSChannel::A2_ch}; // U7 curr_sense_board

constexpr uint8_t LEAD_SCREW_ENC_MUX = 4;
constexpr uint8_t ACT_ENC_MUX = 3;

void update(int32_t &act_right_curr, int32_t &lead_screw_curr, float &act_angle,
            float &lead_screw_angle) {
    // act_right_curr = act_right_curr_sense.read();
    // lead_screw_curr = lead_screw_curr_sense.read();

    lead_screw_angle = EncoderBus::read_enc(LEAD_SCREW_ENC_MUX);
    act_angle = EncoderBus::read_enc(ACT_ENC_MUX);
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

CurrentSensor left_curr_sense{&adc0, ADSChannel::A2_ch};  // U3 curr_sense_board
CurrentSensor right_curr_sense{&adc0, ADSChannel::A0_ch}; // U1 curr_sense_board

constexpr uint8_t DRIVE_RIGHT_MUX = 0;
constexpr uint8_t DRIVE_LEFT_MUX = 1;

void update(int32_t &left_curr, int32_t &right_curr, float &left_angle,
            float &right_angle) {
    // left_curr = left_curr_sense.read();
    // right_curr = right_curr_sense.read();

    left_angle = EncoderBus::read_enc(DRIVE_LEFT_MUX);
    right_angle = EncoderBus::read_enc(DRIVE_RIGHT_MUX);
}

void cb(int8_t left, int8_t right) {
    // Tank drive steering
    left_drive.write(-left);
    right_drive.write(-right);
}

} // namespace drivetrain

namespace excavation {
STMotorInterface exc_motor{&MC4, STMotor::M1};
CurrentSensor exc_curr_sense{&adc1, ADSChannel::A0_ch}; // U5 curr_sense_board

void update(int32_t &exc_curr) { exc_curr = exc_curr_sense.read(); }

void cb(int8_t speed) { exc_motor.write(speed); }
} // namespace excavation

namespace deposition {
STMotorInterface dep_motor{&MC1, STMotor::M2};
CurrentSensor dep_curr_sense{&adc0, ADSChannel::A1_ch}; // U2 curr_sense_board

constexpr uint8_t DEP_MUX = 2;

void update(int32_t &dep_curr, float &dep_angle) {
    // dep_curr = dep_curr_sense.read();
    dep_angle = EncoderBus::read_enc(DEP_MUX);
}

void cb(int8_t speed) { dep_motor.write(speed); }

} // namespace deposition
