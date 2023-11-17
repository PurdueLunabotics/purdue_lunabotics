#include <algorithm>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>

using namespace std;

float to_rad(float deg) { return deg * M_PI / 180.0; }
float u_mod(float n, float base) {
    float m = fmod(n, base);
    return (m >= 0) ? m : m + base;
}
float ang_delta(float deg, float prev_deg) {
    float raw = deg - prev_deg;
    if (raw == 0) {
        return 0;
    }
    float turn = min(u_mod(raw, 360.), u_mod(-raw, 360.));

    int dir = turn == raw ? abs(raw) / raw : -abs(raw) / raw;

    return turn * dir;
}

struct GearAngle {
  public:
    GearAngle(float gear_ratio) : gear_ratio_(gear_ratio) { cycle_ = 0; }
    float get_rad_from_raw(float deg, float prev_deg) {
        float raw = deg - prev_deg;
        float turn = min(u_mod(raw, 360.), u_mod(-raw, 360.));
        int dir = turn == raw ? abs(raw) / raw : -abs(raw) / raw;
        if (abs(raw) != abs(turn)) { // overflow occured!
            if (raw > 0) {
                cycle_++;
            } else {
                cycle_--;
            }
        }
        return (2 * M_PI * cycle_ + to_rad(deg)) * gear_ratio_;
    }

  private:
    int cycle_;
    float gear_ratio_;
};

float adc_to_current_ACS711_31A(float adc_value, float adc_fsr = 4.096,
                                float vcc = 3.3) {
    float vout = adc_value / pow(2, 16 - 1) * adc_fsr;
    return 73.3 * (vout / vcc) - 36.7;
}

float adc_to_current_ACS711_15A(float adc_value, float adc_fsr = 4.096,
                                float vcc = 3.3) {
    float vout = adc_value / pow(2, 16 - 1) * adc_fsr;
    return 36.7 * (vout / vcc) - 18.3;
}
