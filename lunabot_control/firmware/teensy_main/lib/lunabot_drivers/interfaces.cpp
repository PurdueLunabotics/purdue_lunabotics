#include "interfaces.hpp"

// Stepper Motor Interfacing

StepperInterface::StepperInterface(uint8_t PWM1_P, uint8_t PWM2_P,
                                   uint8_t DIR1_P, uint8_t DIR2_P, int steps,
                                   int speed, int step_size)
    : s_(steps, DIR1_P, DIR2_P), PWM1_P_{PWM1_P}, PWM2_P_{PWM2_P},
      DIR1_P_{DIR1_P}, DIR2_P_{DIR2_P}, en_{0}, steps_{steps}, speed_{speed},
      step_size_{step_size} {
  pinMode(PWM1_P_, OUTPUT);
  pinMode(PWM2_P_, OUTPUT);
  s_.setSpeed(speed_);
}

void StepperInterface::step(StepperDir dir) {
  if (en_) {
    s_.step(step_size_ * dir);
  }
}

void StepperInterface::on() {
  digitalWrite(PWM1_P_, HIGH);
  digitalWrite(PWM2_P_, HIGH);
  en_ = 1;
}

void StepperInterface::off() {
  digitalWrite(PWM1_P_, LOW);
  digitalWrite(PWM2_P_, LOW);
  en_ = 0;
}

// PWM Motor Control Interfacing

MotorInterface::MotorInterface(uint8_t PWM_P, uint8_t DIR_P)
    : PWM_P_{PWM_P}, DIR_P_{DIR_P} {
  pinMode(DIR_P_, OUTPUT);
  pinMode(PWM_P_, OUTPUT);
}

void MotorInterface::write(uint8_t pwm, MotorDir dir) {
  digitalWrite(DIR_P_, dir);
  analogWrite(PWM_P_, pwm);
}

// Sabertooth MC Interfacing

int STMotorInterface::initialized_serial_ = 0;

STMotorInterface::STMotorInterface(Sabertooth *s, STMotor m)
    : st_{s}, motor_{m} {}

void STMotorInterface::init_serial(HardwareSerial s, int baud_rate) {
  s.begin(baud_rate);
  initialized_serial_ = 1;
}
void STMotorInterface::write(int8_t power) {
  if (initialized_serial_) {
    power = min(power, 127);
    st_->motor(static_cast<byte>(motor_), power);
  }
}

// ---- Sensors ----

// Current Sensor
//
int CurrentSensor::initialized_ = 0;

CurrentSensor::CurrentSensor(ADS1115_lite *adc, ADSChannel ch)
    : adc_{adc}, ch_{ch}, curr_{-1} {}

void CurrentSensor::init_ads1115(ADS1115_lite *adc0, ADS1115_lite *adc1) {
  adc0->setGain(ADS1115_REG_CONFIG_PGA_4_096V); //  +/-4.096V range = Gain 2
  adc0->setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); // 128 SPS, or every 7.8ms

  adc1->setGain(ADS1115_REG_CONFIG_PGA_4_096V); //  +/-4.096V range = Gain 2
  adc1->setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); // 128 SPS, or every 7.8ms

  if (!adc0->testConnection() || !adc1->testConnection()) {
    return;
  }
  initialized_ = 1;
}

int16_t CurrentSensor::read() { return curr_; }

void CurrentSensor::loop() {
  // The mux setting must be set every time each channel is read, there is NOT
  // a separate function call for each possible mux combination.
  if (initialized_) {
    adc_->setMux(ch_);         // Set mux
    adc_->triggerConversion(); // Start a conversion.  This immediatly returns
    while (!adc_->isConversionDone())
      ;
    curr_ =
        adc_->getConversion(); // This polls the ADS1115 and wait for
                               // conversion to finish, THEN returns the value
  }
}
