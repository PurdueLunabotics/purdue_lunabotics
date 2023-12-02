#include <algorithm>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>

using namespace std;

#define DEG2RAD(x) ((x)*M_PI / 180.0)

float u_mod(float n, float base) {
  float m = fmod(n, base);
  return (m >= 0) ? m : m + base;
}

float deg_angle_delta(float deg, float prev_deg) {
  float raw = deg - prev_deg;
  if (raw == 0) {
    return 0;
  }
  float turn = min(u_mod(raw, 360.), u_mod(-raw, 360.));
  int dir = turn == raw ? abs(raw) / raw : -abs(raw) / raw;
  return turn * dir;
}

void angle_noise_rej_filter(float &curr_angle_deg, float &prev_angle_deg, float max_delta) {
  float delta = deg_angle_delta(curr_angle_deg, prev_angle_deg);
  if (abs(delta) >= max_delta) {
    curr_angle_deg = prev_angle_deg;
  } else {
    prev_angle_deg = curr_angle_deg;
  }
}

float adc_to_current_ACS711_31A(float adc_value, float adc_fsr = 4.096, float vcc = 3.3) {
  float vout = adc_value / pow(2, 16 - 1) * adc_fsr;
  return 73.3 * (vout / vcc) - 36.7;
}

float adc_to_current_ACS711_15A(float adc_value, float adc_fsr = 4.096, float vcc = 3.3) {
  float vout = adc_value / pow(2, 16 - 1) * adc_fsr;
  return 36.7 * (vout / vcc) - 18.3;
}
