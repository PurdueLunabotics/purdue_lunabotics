#include <algorithm>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <array>

using namespace std;

#define DEG2RAD(x) ((x)*M_PI / 180.0)

template <size_t N>
struct RingBuffer {

  void push(float value) {
    sum -= data[head];
    sum += value;
    data[head] = value;
    head = (head + 1) % data.size();
  }

  float mean() { return sum / data.size(); }

  std::array<float,N> data;
  int head;
  float sum;
};

float u_mod(float n, float base) {
  float m = fmod(n, base);
  return (m >= 0) ? m : m + base;
}

// TODO: Fix cyclic spikes in delta
float deg_angle_delta(float deg, float prev_deg) {
  float raw = deg - prev_deg;
  if (raw == 0) {
    return 0;
  }
  float turn = min(u_mod(raw, 360.), u_mod(-raw, 360.));
  int dir = abs(raw) / raw;
  return turn * dir;
}

// alpha = 0, no filter, alpha = 1, full
float leaky_integrator(float value, float prev_filter_value, float alpha) {
  return alpha * prev_filter_value + (1 - alpha) * value;
}

void angle_noise_rej_filter(float *curr_angle_deg, float *prev_angle_deg, float max_delta) {
  float delta = deg_angle_delta(*curr_angle_deg, *prev_angle_deg);
  if (abs(delta) >= max_delta) {
    *curr_angle_deg = *prev_angle_deg;
  } else {
    *prev_angle_deg = *curr_angle_deg;
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
