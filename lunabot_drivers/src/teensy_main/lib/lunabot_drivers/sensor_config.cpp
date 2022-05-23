#include "sensor_config.h"

void init_hall(uint8_t pin, void (*cb)(), State *sensor)
{
    pinMode(pin, INPUT_PULLUP);
    attachInterrupt(pin, cb, FALLING);
    sensor->state = sensor->init_state;
}
