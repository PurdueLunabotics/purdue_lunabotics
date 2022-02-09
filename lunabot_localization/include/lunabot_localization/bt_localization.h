#pragma once

#include <string.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <cmath>

#include <lunabot_localization/bt_manager.h>
#include <lunabot_localization/laterate.h>

class BTLocalization 
{
private:
    //UUIDs of BLE beacons
    float r1, r2, r3;
    float k1, k2, k3;

    std::unique_ptr<BTManager> manager;
    
    float rssiConv(float s);
    float fastInvSqrt(float x);
    float calcRadius(float s, float k);
    void getRadii(float s1, float s2, float s3);
    void getStrs(float* s1, float* s2, float* s3);
public:
    BTLocalization(uint128_t UUID1_, uint128_t UUID2_, uint128_t UUID3_);
    float x, y, z = 0;
    void getPos();
    void calibrate(float r1, float r2, float r3);
};
