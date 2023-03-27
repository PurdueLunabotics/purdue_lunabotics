#include <robot.hpp>

Sabertooth MC1(128, ST_SERIAL); // top
Sabertooth MC2(129, ST_SERIAL); // middle
Sabertooth MC3(130, ST_SERIAL); // bottom
Sabertooth MC4(131, ST_SERIAL); // high-current

namespace actuation {
STMotorInterface act_right{&MC2, STMotor::M1};
STMotorInterface act_left{&MC2, STMotor::M2};
STMotorInterface lead_screw_motor{&MC1, STMotor::M1};

void cb(int8_t lead_screw, int8_t lin_act) {
  act_left.write(-lin_act);
  act_right.write(lin_act);
  lead_screw_motor.write(lead_screw);
}

} // namespace actuation

namespace drivetrain {
STMotorInterface left_drive{&MC3, STMotor::M1};
STMotorInterface right_drive{&MC3, STMotor::M2};
void cb(int8_t left, int8_t right) {
  // Tank drive steering
  left_drive.write(-left);
  right_drive.write(-right);
}
} // namespace drivetrain

namespace excavation {
STMotorInterface exc_motor{&MC4, STMotor::M1};
void cb(int8_t speed) { exc_motor.write(speed); }
} // namespace excavation

namespace deposition {
STMotorInterface dep_motor{&MC1, STMotor::M2};
void cb(int8_t speed) { dep_motor.write(speed); }
} // namespace deposition
