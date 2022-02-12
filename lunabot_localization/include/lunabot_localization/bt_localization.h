#pragma once

#include <string.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <cmath>

#include <geometry_msgs/Point.h>

#include <lunabot_localization/bt_manager.h>
#include <lunabot_localization/laterate.h>

class BTLocalization 
{
private:
    //UUIDs of BLE beacons
    float r1, r2, r3;
    float k1, k2, k3;

    vector::vector b1, b2, b3; //Basis
    vector::vector A, B, C; //Beacon positions
    geometry_msgs::vector bAnchor; //Basis anchor - points to "U" from the robot

    std::unique_ptr<BTManager> manager;
    
    float rssiConv(float s);
    float fastInvSqrt(float x);
    float calcRadius(float s, float k);
    void getRadii(float s1, float s2, float s3);
    void getStrs(float* s1, float* s2, float* s3);
public:
    BTLocalization(uint128_t UUID1_, uint128_t UUID2_, uint128_t UUID3_, geometry_msgs::Point U, geometry_msgs::Point V, geometry_msgs::Point W);
    float x, y, z = 0;
    void getPos();
    void calibrate(float r1, float r2, float r3);
};

struct vector
{
    vector(float x_, float y_, float z_);
    
    float x, y, z = 0;
}