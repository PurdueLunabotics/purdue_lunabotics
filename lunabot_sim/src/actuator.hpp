#pragma once

#include <mujoco/mjtnum.h>
#include <string>
#include "mujoco/mujoco.h"

class Actuator {
    private:
        mjModel *model;
        mjData *data;
        int idx;
    public:
        Actuator(std::string name, mjModel *model, mjData *data);
        Actuator();

        void ctrl(mjtNum value);
};
