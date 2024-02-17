#include "interfaces.hpp"

// Stepper Motor Interfacing

Stepper2Phase_MotorCtrl::Stepper2Phase_MotorCtrl(uint8_t PWM1_P, uint8_t PWM2_P, uint8_t DIR1_P,
                                                 uint8_t DIR2_P, int steps, int speed,
                                                 int step_size)
    : s_(steps, DIR1_P, DIR2_P), PWM1_P_{PWM1_P}, PWM2_P_{PWM2_P}, DIR1_P_{DIR1_P}, DIR2_P_{DIR2_P},
      enabled_{0}, steps_{steps}, speed_{speed}, step_size_{step_size} {
  pinMode(PWM1_P_, OUTPUT);
  pinMode(PWM2_P_, OUTPUT);
  s_.setSpeed(speed_);
}

void Stepper2Phase_MotorCtrl::step(StepperDir dir) {
  if (enabled_) {
    s_.step(step_size_ * dir);
  }
}

void Stepper2Phase_MotorCtrl::on() {
  digitalWrite(PWM1_P_, HIGH);
  digitalWrite(PWM2_P_, HIGH);
  enabled_ = 1;
}

void Stepper2Phase_MotorCtrl::off() {
  digitalWrite(PWM1_P_, LOW);
  digitalWrite(PWM2_P_, LOW);
  enabled_ = 0;
}

// PWM Motor Control Interfacing

PWM_MotorCtrl::PWM_MotorCtrl(uint8_t PWM_P, uint8_t DIR_P) : PWM_P_{PWM_P}, DIR_P_{DIR_P} {
  pinMode(DIR_P_, OUTPUT);
  pinMode(PWM_P_, OUTPUT);
}

void PWM_MotorCtrl::write(uint8_t pwm, MotorDir dir) {
  digitalWrite(DIR_P_, dir);
  analogWrite(PWM_P_, pwm);
}

// Sabertooth MC Interfacing

int Sabertooth_MotorCtrl::initialized_serial_ = 0;

Sabertooth_MotorCtrl::Sabertooth_MotorCtrl(Sabertooth *s, STMotor m) : st_{s}, motor_{m} {}

void Sabertooth_MotorCtrl::init_serial(HardwareSerial s, int baud_rate) {
  s.begin(baud_rate);
  initialized_serial_ = 1;
}
void Sabertooth_MotorCtrl::write(int8_t power) {
  if (initialized_serial_) {
    power = min(power, 127);
    st_->motor(static_cast<byte>(motor_), power);
  }
}

// ---- Sensors ----

// Current Sensor

const int ACS711_Current_Bus::ADS_CHANNELS[ACS711_Current_Bus::CH_SIZE] = {
    ADS1115_REG_CONFIG_MUX_SINGLE_0, ADS1115_REG_CONFIG_MUX_SINGLE_1,
    ADS1115_REG_CONFIG_MUX_SINGLE_2, ADS1115_REG_CONFIG_MUX_SINGLE_3};

ADS1115_lite ACS711_Current_Bus::adc0_(ADS1115_ADDRESS_ADDR_SCL);
ADS1115_lite ACS711_Current_Bus::adc1_(ADS1115_ADDRESS_ADDR_SDA);

int ACS711_Current_Bus::initialized_ = 0;
int16_t ACS711_Current_Bus::curr_buffer_[ACS711_Current_Bus::BUSES][ACS711_Current_Bus::BUS_SIZE] =
    {-1};
uint8_t ACS711_Current_Bus::adc0_ch_ = 0;
uint8_t ACS711_Current_Bus::adc1_ch_ = 0;

void ACS711_Current_Bus::init_ads1115() {
  adc0_.setGain(ADS1115_REG_CONFIG_PGA_4_096V);      //  +/-4.096V range = Gain 2
  adc0_.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); // 128 SPS, or every 7.8ms

  adc1_.setGain(ADS1115_REG_CONFIG_PGA_4_096V);      //  +/-4.096V range = Gain 2
  adc1_.setSampleRate(ADS1115_REG_CONFIG_DR_128SPS); // 128 SPS, or every 7.8ms

  if (!adc0_.testConnection() || !adc1_.testConnection()) {
    return;
  }

  adc0_.triggerConversion();
  adc1_.triggerConversion();
  initialized_ = 1;
}

void ACS711_Current_Bus::transfer() {
  if (initialized_) {
    if (adc0_.isConversionDone()) {
      curr_buffer_[0][adc0_ch_] = adc0_.getConversion(); // This polls the ADS1115 and wait for
                                                         // conversion to finish, THEN returns the
                                                         // value
      adc0_ch_ = (adc0_ch_ + 1) % CH_SIZE;
      adc0_.setMux(ADS_CHANNELS[adc0_ch_]); // Set mux
      adc0_.triggerConversion();            // Start a conversion.  This immediatly
    }

    if (adc1_.isConversionDone()) {
      curr_buffer_[1][adc1_ch_] = adc1_.getConversion(); // This polls the ADS1115 and wait for
                                                         // conversion to finish, THEN returns the
                                                         // value
      adc1_ch_ = (adc1_ch_ + 1) % CH_SIZE;
      adc1_.setMux(ADS_CHANNELS[adc0_ch_]); // Set mux
      adc1_.triggerConversion();            // Start a conversion.  This immediatly
    }
  }
}

int16_t ACS711_Current_Bus::read(uint8_t bus, uint8_t mux) { return curr_buffer_[bus][mux]; }

// VLH35_Angles

#define CLOCK_SPEED 100'000u // 100kHz SSI Clock

SPISettings spi_settings(CLOCK_SPEED, MSBFIRST, SPI_MODE1);

volatile uint8_t VLH35_Angle_Bus::curr_id_ = 0;
volatile uint8_t VLH35_Angle_Bus::spi_buffer_[VLH35_Angle_Bus::BUFFER_SIZE] = {0};
volatile uint32_t VLH35_Angle_Bus::enc_buffer_[VLH35_Angle_Bus::BUS_SIZE] = {0};

void VLH35_Angle_Bus::init() {
  SPI.begin();
  pinMode(clk_p_, OUTPUT);
  pinMode(sel0_p_, OUTPUT);
  pinMode(sel1_p_, OUTPUT);
  pinMode(sel2_p_, OUTPUT);
  select_enc_(curr_id_);
  digitalWriteFast(clk_p_,
                   HIGH); // Set CLK line HIGH (to meet the requirements
                          // of SSI interface)
}

void VLH35_Angle_Bus::select_enc_(uint8_t id) {
  uint8_t sel0 = id & 1;
  uint8_t sel1 = id >> 1 & 1;
  uint8_t sel2 = id >> 2 & 1;
  digitalWrite(sel0_p_, sel0);
  digitalWrite(sel1_p_, sel1);
  digitalWrite(sel2_p_, sel2);
}

void VLH35_Angle_Bus::transfer() {
  digitalWriteFast(clk_p_,
                   LOW); // Set CLK line LOW (to inform encoder -> latch data)

  // Before in setup() the pinMode() change the CLK pin function to
  // output, now we have to enable usage of this pin by SPI port with
  // calling SPI.begin():
  SPI.begin();
  SPI.beginTransaction(spi_settings); // We use transactional API

  for (int i = 0; i < BUFFER_SIZE; i++) {
    spi_buffer_[i] = SPI.transfer(0xAA); // Transfer anything and read data back
  }

  SPI.endTransaction(); // We use transactional API

  pinMode(clk_p_,
          OUTPUT);            // A while before we set CLK pin to be used by SPI
                              // port, now we have to change it manually...
  digitalWrite(clk_p_, HIGH); // ... back to idle HIGH

  delayMicroseconds(27); // Running above 500kHz perform Delay First Clock function

  enc_buffer_[curr_id_] = static_cast<uint32_t>(spi_buffer_[0] << 8) |
                          static_cast<uint32_t>(spi_buffer_[1]); // here transfer8 was made

  curr_id_ = (curr_id_ + 1) % BUS_SIZE;
  select_enc_(curr_id_);
}

float VLH35_Angle_Bus::read_enc(uint8_t id) {
  uint32_t data;
  noInterrupts();
  data = enc_buffer_[id];
  interrupts();

  // Shift one bit right - (MSB of position was placed on at MSB of
  // uint32_t, so make it right and shift to make MSB position at 30'th
  // bit): data = data >> 1;

  // encoderPosition is placed in front (starting from MSB of uint32_t) so
  // shift it for 11 bits to align position data right

  return static_cast<float>(data) / static_cast<float>(1 << 16) * 360.0F;
}

volatile float M5Stack_UWB_Trncvr::recv_buffer_[NUM_UWB_TAGS] = {0};

float M5Stack_UWB_Trncvr::read_uwb(uint8_t id) {
  noInterrupts();
  float data = recv_buffer_[id];
  interrupts();
  return data;
}

void M5Stack_UWB_Trncvr::init() {
  UWBSerial.begin(115200);

  // serial 2 setup as tag on bot (does dist calc)
  for (int b = 0; b < 2; b++) { // Repeat twice to stabilize the connection
    delay(50);
    UWBSerial.write("AT+anchor_tag=0\r\n"); // Set up the Tag
    delay(50);
    UWBSerial.write("AT+interval=5\r\n");  // Set the calculation precision,
                                           // the larger the response is, the
                                           // slower it will be
    delay(50);                             // 设置计算精度，越大响应越慢
    UWBSerial.write("AT+switchdis=1\r\n"); // Began to distance 开始测距
    delay(50);
    if (b == 0) {
      UWBSerial.write("AT+RST\r\n"); // RESET 复位
    }
  }
  if (UWBSerial.available()) {
    delay(3);
  }
}

void M5Stack_UWB_Trncvr::transfer() {
  if (UWBSerial.available()) {
    // Read the original input
    String originalInput = UWBSerial.readStringUntil('\n');
    originalInput.trim();

    // Check if the message starts with "anX:" where X is a digit
    if (originalInput.startsWith("an") && isdigit(originalInput.charAt(2)) &&
        originalInput.charAt(3) == ':') {
      // Extract sensor number
      int sensorNumber = originalInput.charAt(2) - '0';

      // Find the index of the colon
      int colonIndex = originalInput.indexOf(':');

      // Extract the value part and trim spaces
      String valueString = originalInput.substring(colonIndex + 1);
      valueString.trim();

      // Convert the trimmed string to a float
      float num = valueString.toFloat();

      // Assign the values to d0, d1, or d2 based on sensor number
      M5Stack_UWB_Trncvr::recv_buffer_[sensorNumber] = num;
    }
  }
}

void KillSwitchRelay::setup() { 
  pinMode(kill_pin, OUTPUT); 
  reset();
}

void KillSwitchRelay::reset() { 
  digitalWrite(kill_pin, HIGH); 
  dead = false;
}

void KillSwitchRelay::kill() { 
  digitalWrite(kill_pin, LOW);
  dead = true;
}

void KillSwitchRelay::logic(RobotSensors state) {
    //check for high current

  if (state.exc_curr >= kill_cur) {
    kill()
  }
  if (state.dep_curr >= kill_cur) {
    kill()
  }
  if (state.drive_left_curr >= kill_cur) {
    kill()
  }
  if (state.drive_right_curr >= kill_cur) {
    kill()
  }
  if (state.act_right_curr >= kill_cur) {
    kill()
  }
  if (state.lead_screw_curr_curr >= kill_cur) {
    kill()
  }

  if (dead) {
    reset();
  }

}
