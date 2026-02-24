#include "interfaces.hpp"
#include "ADS1119.h"
#include "wiring.h"

// Sabertooth MC Interfacing

int Sabertooth_MotorCtrl::initialized_serial_ = 0;

Sabertooth_MotorCtrl::Sabertooth_MotorCtrl(Sabertooth *s, STMotor m) : st_{s}, motor_{m} {}

void Sabertooth_MotorCtrl::init_serial(HardwareSerial &s, int baud_rate) {
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

ADS1119Configuration ADS1119_Current_Bus::configuration = {};
ADS1119 ADS1119_Current_Bus::ads1 = ADS1119(ads1_addr);

void ADS1119_Current_Bus::init_ads1119() {
  configuration.mux = ADS1119MuxConfiguration::positiveAIN0negativeAIN1;
  configuration.gain = ADS1119Configuration::Gain::one;
  configuration.dataRate = ADS1119Configuration::DataRate::sps20;
  configuration.conversionMode = ADS1119Configuration::ConversionMode::continuous;
  configuration.voltageReference = ADS1119Configuration::VoltageReferenceSource::external;
  configuration.externalReferenceVoltage = 3.305;

    ads1.begin(&configuration);
  /* Config ADS1119 Amux Input as Single Ended*/
  ads1.configADCSingleEnded();
  /* Select ADS1119 Channel
  Single Ended: 4 CHANNELS => AN0, AN1, AN2, AN3
  Differential: 3 CHANNELS => AN0-AN1, AN2-AN3, AN1-AN2,
  */
  ads1.selectChannel(0); // select AN0 (single ended input mode)
  ads1.reset();
}

float ADS1119_Current_Bus::read(uint8_t mux) {
  //ports are zero and
  ads1.selectChannel(mux);
  return ADS1119_Current_Bus::adc_to_current_31A(ads1.readVoltage());
}

float ADS1119_Current_Bus::adc_to_current_31A(float adc_value, float adc_fsr, float vcc) {
  float vout = adc_value / pow(2, 2) * adc_fsr;
  return 73.3 * (vout / vcc) - 37.52;
}

long KillSwitchRelay::kill_time;
bool KillSwitchRelay::dead;

void KillSwitchRelay::init() {
  pinMode(kill_pin, OUTPUT);
  KillSwitchRelay::dead = false;
  reset();
  KillSwitchRelay::kill_time = millis();
}

void KillSwitchRelay::reset() {
  digitalWrite(kill_pin, HIGH);
  KillSwitchRelay::dead = false;
}

void KillSwitchRelay::kill() {
  digitalWrite(kill_pin, LOW);
  KillSwitchRelay::kill_time = millis();
  KillSwitchRelay::dead = true;
}

void KillSwitchRelay::disable_motor(int id, RobotEffort &effort) {
  switch (id) {
  case 0:
    effort.excavate = 0;
    break;
  case 1:
    effort.deposit = 0;
    break;
  case 2:
    effort.left_drive = 0;
    break;
  case 3:
    effort.right_drive = 0;
    break;
  default:
    return;
  }
}

// exc, dep, drive_L, drive_R
volatile int KillSwitchRelay::cutoff_buffer[4] = {0};
volatile int KillSwitchRelay::disable_counter[4] = {0};
volatile bool KillSwitchRelay::is_disable[4] = {false};

void KillSwitchRelay::logic(RobotEffort &effort) {
  /*
  if (KillSwitchRelay::dead && millis() - KillSwitchRelay::kill_time >= relay_dead_time) {
    reset();
  } */

  float exc_curr = ADS1119_Current_Bus::adc_to_current_31A(excavation::update_curr());
  float dep_curr = ADS1119_Current_Bus::adc_to_current_31A(deposition::update_curr());
  float drive_left_curr = ADS1119_Current_Bus::adc_to_current_31A(drivetrain::update_curr_left());
  float drive_right_curr = ADS1119_Current_Bus::adc_to_current_31A(drivetrain::update_curr_right());

  if (exc_curr >= exdep_kill_curr) {
    cutoff_buffer[0] += cutoff_increase;
  }
  if (dep_curr >= exdep_kill_curr) {
    cutoff_buffer[1] += cutoff_increase;
  }
  if (drive_left_curr >= drive_kill_curr) {
    cutoff_buffer[2] += cutoff_increase;
  }
  if (drive_right_curr >= drive_kill_curr) {
    cutoff_buffer[3] += cutoff_increase;
  }

  for (int i = 0; i < 4; ++i) {
    cutoff_buffer[i] -= cutoff_decay;
    if (cutoff_buffer[i] < 0) {
      cutoff_buffer[i] = 0;
    }
    if (is_disable[i]) {
      if (cutoff_buffer[i] >= reset_thresh) {
        disable_motor(i, effort);
      } else {
        is_disable[i] = false;
      }
    } else {
      if (cutoff_buffer[i] >= cutoff_thresh) {
        disable_motor(i, effort);
        is_disable[i] = true;
        disable_counter[i] += 1;
      }
    }

    if (disable_counter[i] >= kill_thresh) {
      disable_counter[i] = 0;
      kill();
    }
  }
}

CRGB Led_Strip::all_led[Led_Strip::NUM_LEDS];

void Led_Strip::init() {
  FastLED.addLeds<WS2812B, 6, GRB>(Led_Strip::all_led, Led_Strip::NUM_LEDS);
  FastLED.setBrightness(Led_Strip::BRIGHTNESS);
}

void Led_Strip::set_color(int32_t color_in) {
  CRGB color_choice;

  switch (color_in) {
  case 0:
    color_choice = CRGB::Black;
    break;
  case 1:
    color_choice = CRGB::Red;
    break;
  case 2:
    color_choice = CRGB::Green;
    break;
  case 3:
    color_choice = CRGB::Blue;
    break;
  case 4:
    color_choice = CRGB::White;
    break;
  case 5:
    color_choice = CRGB::Yellow;
    break;
  case 6:
    color_choice = CRGB::Aqua;
    break;
  case 7:
    color_choice = CRGB::Magenta;
    break;
  default:
    color_choice = CRGB::Black;
    break;
  }

  for (int i = 0; i < NUM_LEDS; ++i) {
    Led_Strip::all_led[i] = color_choice;
  }
  FastLED.show();
}

Encoder Encoder_Bus::encs[NUM_ACTUATORS] = {
    Encoder(PIN_LIST[0], PIN_LIST[1]),
    Encoder(PIN_LIST[2], PIN_LIST[3]),
};

void Encoder_Bus::init() {
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    encs[i].write(0);
  }
}

long Encoder_Bus::read(uint8_t id) {
  // returns the count since last read, and resets the count to 0
  if (id != 1 && id != 0) {
      return -1;
  }
  long val = encs[id].read();
  return val;
}
