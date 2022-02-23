#include <lunabot_localization/bt_localization.h>

BTLocalization::BTLocalization() 
{
    this->manager.reset(new BTManager);
}

float BTLocalization::fastInvSqrt(float x) 
{
    //Fast inverse square root (see https://en.wikipedia.org/wiki/Fast_inverse_square_root#Aliasing_to_an_integer_as_an_approximate_logarithm)
    unsigned long i;
    float x2, y;
    const float threehalfs = 1.5F;
    x2 = x * 0.5F;
    y = x;
    i = * ( unsigned long * ) &y;
    i = 0x5F1FFFF9ul - (i >> 1); //See "Subsequent improvements", ibid.
    y = * ( float * ) &i;
    y *= 0.703952253f * (2.38924456f - x * y * y);
    return y;
}

float BTLocalization::calcRadius(float s, float k) 
{
    return k * fastInvSqrt(s);
}

void BTLocalization::getRadii(float s1, float s2, float s3) 
{
    this->r1 = calcRadius(s1, this->k1);
    this->r2 = calcRadius(s2, this->k2);
    this->r3 = calcRadius(s3, this->k3);
}

void BTLocalization::getPos() 
{
    std::vector<btevent> events = this->manager->getResults();
    float s1, s2, s3; //I am assuming that RSSI scales linearly- needs testing
    for (btevent event : events) 
    {
        switch(event.r_id) 
        {
        case 1:
            s1 = event.rssi;
            break;
        case 2:
            s2 = event.rssi;
            break;
        case 3:
            s3 = event.rssi;
            break;
        }
    }
    getRadii(s1, s2, s3);

    multilaterate(&this->x, &this->y, &this->z, this->r1, this->r2, this->r3, this->x, this->y, this->z, 0);
}

//This is where ROS wrappers should attach!
