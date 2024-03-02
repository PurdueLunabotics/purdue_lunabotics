#include <robot.hpp>

Sabertooth MC1(128, ST_SERIAL);
Sabertooth MC2(129, ST_SERIAL);

namespace actuation {
//static StepperDir lead_screw_dir;
//StepperInterface lead_screw{19, 22, 20, 23, 200, 90, 30};
STMotorInterface lin_acts{&MC2, STMotor::M1};

//void run() { lead_screw.step(lead_screw_dir); }

void cb(int8_t lead_screw_pwm, int8_t lin_act_pwm) {
    //lead_screw_dir = (lead_screw_pwm > 0) ? EXTEND : RETRACT;
    lin_acts.write(lin_act_pwm);
    //if (lead_screw_pwm != 0) {
    //    lead_screw.on();
    //} else {
    //    lead_screw.off();
    //}
}
} // namespace actuation

namespace drivetrain {
STMotorInterface left_drive{&MC1, STMotor::M1};
STMotorInterface right_drive{&MC1, STMotor::M2};
void cb(int8_t left, int8_t right) {
    // Tank drive steering
    left_drive.write(left);
    right_drive.write(right);
}
} // namespace drivetrain

namespace excavation {
STMotorInterface exc_motor{&MC2, STMotor::M1};
void cb(int8_t speed) { exc_motor.write(speed); }
} // namespace excavation

namespace deposition {

MotorInterface dep_motor{13, 14};

void cb(int8_t speed) {
    MotorDir dep_dir =
        (speed > 0)
            ? CCW
            : CW; // CCW rotates deposition up, CW retracts deposition down
    dep_motor.write(speed, dep_dir);
}
} // namespace deposition
