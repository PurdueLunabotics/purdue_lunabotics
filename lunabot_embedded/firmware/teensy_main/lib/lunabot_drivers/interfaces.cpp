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
  ads1.selectChannel(2); // select AN0 (single ended input mode)
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


CRGB Led_Strip::all_led[NUM_LEDS];

void Led_Strip::init() {
  FastLED.addLeds<WS2812B, 4, GRB>(Led_Strip::all_led, NUM_LEDS);
  FastLED.setBrightness(Led_Strip::BRIGHTNESS);
}

void Led_Strip::set_color(int32_t color_in, uint8_t counter) {

  int MAX_DIGITS = 9; // how many digits are allowed in this integer (each digit is one color)

  CRGB RAINBOW_MAGIC_WORD = 0xDEDEDE;

  int digits[MAX_DIGITS] = {0};

  int nonzero_digits = 0; // how many digits are actually holding a color
  for (int i = 0; i < MAX_DIGITS; i++) {
    digits[i] = color_in % 10;
    color_in /= 10;

    if (digits[i] != 0) {
      nonzero_digits++;
    }
  }

  // if we get all zeroes, label it as one color (0 / OFF)
  if (nonzero_digits == 0) {
    nonzero_digits = 1;
  }

  CRGB colors[MAX_DIGITS];

  // fill in the array of colors with CRGB data
  for (int i = 0; i < nonzero_digits; i++) {
    CRGB color;
    switch(digits[i]) {
      case 0:
        color = CRGB::Black;
        break;
      case 1:
        color = CRGB::Red;
        break;
      case 2:
        color = 0xFF5000; // Orange but better
        break;
      case 3:
        color = CRGB::Gold;
        break;
      case 4:
        color = CRGB::Green;
        break;
      case 5:
        color = 0x00FF4F; // Teal
        break;
      case 6:
        color = CRGB::Blue;
        break;
      case 7:
        color = CRGB::Magenta;
        break;
      case 8:
        color = CRGB::White;
        break;
      case 9:
        color = RAINBOW_MAGIC_WORD;
        break;
      default:
        color = CRGB::Black;
        break;
    }
    colors[i] = color;
  }


  int group_size = NUM_LEDS / nonzero_digits;
  int current_LED = 0;
  for (int i = 0; i < nonzero_digits; i++) {
    for (int j = 0; j < group_size; j++) {

      if (current_LED >= NUM_LEDS) {
        break;
      }

      Led_Strip::all_led[current_LED] = colors[i];

      // TODO: check if this works
      if (colors[i] == RAINBOW_MAGIC_WORD) {
        uint8_t h = counter;
        int s = 255;
        int v = 255;

        h += (int) (((float) j / group_size) * 255);

        CHSV hsv(h, s, v);
        CRGB color;
        hsv2rgb_rainbow(hsv, color);
        Led_Strip::all_led[current_LED] = color;
      }

      current_LED++;

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
