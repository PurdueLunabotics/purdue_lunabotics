#include "sensor_config.h"
0
void init_hall(uint8_t pin, void (*cb)())
{
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(pin, cb, CHANGE);
}
