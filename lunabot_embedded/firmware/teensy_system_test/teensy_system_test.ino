// include ROS and all messages
#include <interfaces.hpp>

#define ST_SERIAL Serial1
#define ST_BAUD_RATE 9600

Sabertooth MC1(128, ST_SERIAL);
Sabertooth MC2(129, ST_SERIAL);
Sabertooth MC3(130, ST_SERIAL);
Sabertooth MC4(131, ST_SERIAL);

STMotorInterface MC1_M1{&MC1, STMotor::M1};
STMotorInterface MC1_M2{&MC1, STMotor::M1};

STMotorInterface MC2_M1{&MC2, STMotor::M1};
STMotorInterface MC2_M2{&MC2, STMotor::M2};

STMotorInterface MC3_M1{&MC3, STMotor::M1};
STMotorInterface MC3_M2{&MC3, STMotor::M2};

STMotorInterface MC4_M1{&MC4, STMotor::M1};
STMotorInterface MC4_M2{&MC4, STMotor::M2};

void setup() { STMotorInterface::init_serial(ST_SERIAL, ST_BAUD_RATE); }

void loop() {
    MC4_M1.write(125);
    delay(1000);
    MC4_M1.write(-125);
    delay(1000);
}
