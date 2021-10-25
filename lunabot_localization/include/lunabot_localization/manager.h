#pragma once

#include "laterate.cpp"
#include "bluetooth_manage.cpp"

class Manager
{
private:
    float r1, r2, r3;
    float k1;
    float k2;
    float k3;
    HCIScanner * scanner;
    float fastInvSqrt(float x);
    float calcRadius(float s, float k);
    void getRadii(float s1, float s2, float s3);

public:
    Manager();
    float x, y, z = 0;
    void getPos(float* x, float* y, float* z);
    
};