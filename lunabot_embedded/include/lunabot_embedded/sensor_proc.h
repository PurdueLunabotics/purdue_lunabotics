#include <algorithm>
#include <array>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <chrono>


using namespace std;

#define DEG2RAD(x) ((x)*M_PI / 180.0)

class StampedValue {
  public:
  uint64_t utimestamp; //micros
  StampedValue(float value) {
    _val = value;
    utimestamp = chrono::duration_cast<std::chrono::microseconds>(
                    chrono::steady_clock::now().time_since_epoch()).count();
  }
  void update(float value) {
    _val = value;
    utimestamp = chrono::duration_cast<std::chrono::microseconds>(
                      chrono::steady_clock::now().time_since_epoch()).count();
  }
  float get() {
    return _val;
  }

  float dt(StampedValue st_val) {
    return (utimestamp - st_val.utimestamp) * 1000000;
  }
  private:
  float _val;
};

template <size_t N> struct RingBuffer {

  void push(float value) {
    sum -= data[head];
    sum += value;
    data[head] = value;
    head = (head + 1) % data.size();
  }

  float mean() { return sum / data.size(); }

  std::array<float, N> data;
  int head;
  float sum;
};

float mod_nonneg(float n, float base) {
  float m = fmod(n, base);
  return (m >= 0) ? m : m + base;
}

float deg_angle_delta(float deg, float prev_deg) {
  float diff = deg - prev_deg;
  diff = mod_nonneg((diff + 180), 360) - 180;
  return diff;
}

// alpha = 0, no filter, alpha = 1, full
float leaky_integrator(float value, float prev_filter_value, float alpha) {
  return alpha * prev_filter_value + (1 - alpha) * value;
}

bool angle_noise_rej_filter(float* curr_raw_angle_deg, float* prev_valid_angle_deg,
                            float max_delta) {
  float delta = deg_angle_delta(*curr_raw_angle_deg, *prev_valid_angle_deg);
  if (abs(delta) >= max_delta) {
    *curr_raw_angle_deg = *prev_valid_angle_deg;
  } else {
    *prev_valid_angle_deg = *curr_raw_angle_deg;
  }

  return abs(delta) >= max_delta; 
}

float adc_to_current_ACS711_31A(float adc_value, float adc_fsr = 4.096, float vcc = 3.3) {
  float vout = adc_value / pow(2, 16 - 1) * adc_fsr;
  return 73.3 * (vout / vcc) - 36.7;
}

float adc_to_current_ACS711_15A(float adc_value, float adc_fsr = 4.096, float vcc = 3.3) {
  float vout = adc_value / pow(2, 16 - 1) * adc_fsr;
  return 36.7 * (vout / vcc) - 18.3;
}
