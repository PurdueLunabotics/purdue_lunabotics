#pragma once

#include <string>
#include "mujoco/mujoco.h"

class Sensor {
    private:
        int idx;
        mjModel *model;
        mjData *data;

    public:
        Sensor(std::string name, mjModel *model, mjData *data);
        Sensor();

        mjtNum* get();
};
