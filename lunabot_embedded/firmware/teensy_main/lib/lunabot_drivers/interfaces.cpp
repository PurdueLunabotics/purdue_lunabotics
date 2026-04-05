#include "interfaces.hpp"
#include "ADS1119.h"
#include "wiring.h"

#define PA01_PULSES_PER_INCH 152
#define PA01_STROKE_LENGTH_INCHES 8
#define INVALID_ID 255
#define INCHES_PER_METER 39.3701

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
  FastLED.addLeds<WS2812B, 4, GRB>(Led_Strip::all_led, Led_Strip::NUM_LEDS);
  FastLED.setBrightness(Led_Strip::BRIGHTNESS);
}

void Led_Strip::set_color(int32_t color_in) {
  CRGB color_choice1;
  CRGB color_choice2;
  int size1 = NUM_LEDS / 2;

  int color1 = color_in % 10;
  int color2 = color_in / 10;

  //minor color
  switch (color1) {
  case 0:
    color_choice1 = CRGB::Black;
    break;
  case 1:
    color_choice1 = CRGB::Yellow;
    break;
  case 2:
    color_choice1 = CRGB::Green;
    break;
  case 3:
    color_choice1 = CRGB::Blue;
    break;
  case 4:
    color_choice1 = CRGB::Magenta;
    break;
  case 5:
    color_choice1 = CRGB::White;
    break;
  case 8:
    //NO PATH
    color_choice1 = CRGB::Orange;
    break;
  case 9:
    //STALL
    color_choice1 = CRGB::Red;
    break;
  default:
    color_choice1 = CRGB::Black;
    break;
  }
  //major color
  switch (color2) {
  case 0:
    color_choice2 = CRGB::Black;
    break;
  case 1:
    //INIT
    color_choice2 = CRGB::Green;
    break;
  case 2:
    //LINKUP
    color_choice2 = CRGB::Gold;
    break;
  case 3:
    //Traversal
    color_choice2 = CRGB::Blue;
    break;
  case 4:
    //DEPOSIT
    color_choice2 = CRGB::Magenta;
    break;
  case 5:
    //EXCAVATE
    color_choice2 = CRGB::White;
    break;
  case 9:
    //MAJOR ERROR
    color_choice2 = CRGB::Red;
    break;
  default:
    color_choice2 = CRGB::Black;
    break;
  }
  if (color_in == 1) {
    int h = 0;
    int s = 255;
    int v = 255;
    for (int i = 0; i < NUM_LEDS; ++i) {
      h += 255.0/NUM_LEDS;
      if (h > 255) {
        h = 0;
      }
      hsv2rgb_rainbow(CHSV(h, s, v), (Led_Strip::all_led[i]));
    }
  }
  else {
    for (int i = 0; i < size1; ++i) {
      Led_Strip::all_led[i] = color_choice1;
    }
    for (int i = size1; i < NUM_LEDS; ++i) {
      Led_Strip::all_led[i] = color_choice2;
    }
  }
  FastLED.show();
}

Encoder Encoder_Bus::encs[NUM_ACTUATORS] = {
    Encoder(PIN_LIST[0], PIN_LIST[1]),
    Encoder(PIN_LIST[2], PIN_LIST[3]),
};

void Encoder_Bus::init(uint8_t option) {
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    if (option) {
        encs[i].write(0);
    } else {
        encs[i].write(PA01_PULSES_PER_INCH * PA01_STROKE_LENGTH_INCHES);
    }
  }
}

long Encoder_Bus::read(uint8_t id) {
  // returns the count since last read, and resets the count to 0
  if (id != 1 && id != 0) {
      return INVALID_ID;
  }
  return encs[id].read();
}
