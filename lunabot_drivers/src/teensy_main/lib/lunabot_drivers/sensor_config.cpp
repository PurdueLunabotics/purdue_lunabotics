#include "sensor_config.h"

void init_sensor(SensorConfig m)
{
    pinMode(m.DIR_P, INPUT);
}