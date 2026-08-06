#include "actuator.hpp"
#include <cstring>
#include <mujoco/mjtnum.h>

Actuator::Actuator() {
    idx = -1;
}

Actuator::Actuator(std::string name, mjModel *model, mjData *data) {
    this->model = model;
    this->data = data;
    for (int i = 0; i < model->nu; i++) {
        const char *act_name = &model->names[model->name_actuatoradr[i]];
        if (strcmp(act_name, name.c_str()) == 0) {
            idx = i;
        }
    }
}

void Actuator::ctrl(mjtNum value) {
    data->ctrl[idx] = value;
}
