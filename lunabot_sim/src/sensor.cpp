#include "sensor.hpp"
#include <cstring>
#include <mujoco/mjtnum.h>

Sensor::Sensor() {
    idx = -1;
}

Sensor::Sensor(std::string name, mjModel *model, mjData *data) {
    this->model = model;
    this->data = data;
    for (int i = 0; i < model->nsensor; i++) {
        const char *sensor_name = &model->names[model->name_sensoradr[i]];
        if (strcmp(sensor_name, name.c_str()) == 0) {
            idx = i;
        }
    }
}

mjtNum *Sensor::get() {
    return &data->sensordata[model->sensor_adr[idx]];
}
