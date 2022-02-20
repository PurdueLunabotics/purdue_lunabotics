#pragma once

#include <string.h>
#include <vector>
#include <chrono>
#include <ctime>
#include <cmath>

#include <lunabot_localization/bt_manager.h>
#include <lunabot_localization/laterate.h>
#include <geometry_msgs/Point.h>

struct Vector
{
    float x, y, z = 0;
    Vector();
    Vector(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vector(geometry_msgs::Point& pt) : x(pt.x), y(pt.y), z(pt.z) {}

    //Vector functions
    float dot(Vector v)
    {
        return this->x * v.x + this->y + v.y + this->z * v.z;
    }

};

class BTLocalization 
{
private:
    //UUIDs of BLE beacons
    float r1, r2, r3;
    float k1, k2, k3;

    Vector b1, b2, b3; //Basis
    Vector A, B, C; //Beacon positions
    Vector bAnchor; //Basis anchor - points to "U" from the robot

    std::unique_ptr<BTManager> manager;
    
    float rssiConv(float s);
    float fastInvSqrt(float x);
    float calcRadius(float s, float k);
    void getRadii(float s1, float s2, float s3);
    void getStrs(float* s1, float* s2, float* s3);
public:
    BTLocalization(uint16_t UUID1_, uint16_t UUID2_, uint16_t UUID3_, 
                    Vector U, Vector V, Vector W);
    float x, y, z = 0;
    void getPos();
    void calibrate(float r1, float r2, float r3);
};