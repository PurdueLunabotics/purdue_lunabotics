#include <lunabot_localization/bt_localization.h>

BTLocalization::BTLocalization(uint16_t UUID1_, uint16_t UUID2_, uint16_t UUID3_, Vector U, Vector V, Vector W) 
{
    this->manager.reset(new BTManager(UUID1_, UUID2_, UUID3_));
    //Convert points to share an "x" value
    //Create first basis vector
    float UVdx = V.x - U.x;
    float UVdy = V.y - U.y;
    float UVdz = V.z - U.z;
    float mUV = sqrt(UVdx * UVdx + UVdy * UVdy + UVdz * UVdz);
    //Create second basis vector
    float UWdx = W.x - U.x;
    float UWdy = W.y - U.y;
    float UWdz = W.z - U.z;
    //Orthogonalize second basis vector
    float dotUVUW = UVdx * UWdx + UVdy * UWdy + UVdz * UWdz;
    UWdz = UWdx - UVdx * dotUVUW;
    UWdy = UWdy - UVdy * dotUVUW;
    UWdz = UWdz - UVdz * dotUVUW;
    float mUW = sqrt(UWdx * UWdx + UWdy * UWdy + UWdz * UWdz);
    //Turn first and second vectors into unit vectors
    UVdx /= mUV; UVdy /= mUV; UVdz /= mUV;
    UWdx /= mUW; UWdy /= mUW; UWdz /- mUW;
    //Take cross product to get third vector, N - it is already orthogonal
    float Ndx = UVdy * UWdz - UVdz * UWdy;
    float Ndy = UVdz * UWdx - UVdx * UWdz;
    float Ndz = UVdx * UWdy - UVdy * UWdx;
    //Store values
    this->bAnchor =  Vector(U.x, U.y, U.z);
    this->bAnchor.x = U.x; this->bAnchor.y = U.y; this->bAnchor.z = U.z;
    this->b1 = Vector(Ndx, Ndy, Ndz);
    this->b2 = Vector(UVdx, UVdy, UVdz);
    this->b3 = Vector(UWdx, UWdy, UWdz);
    //Store beacon positions
    float Ab1 = this->b1.dot(U);
    float Ab2 = this->b2.dot(U);
    float Ab3 = this->b3.dot(U);
    this->A = Vector(Ab1, Ab2, Ab3);
    float Bb1 = this->b1.dot(V);
    float Bb2 = this->b2.dot(V);
    float Bb3 = this->b3.dot(V);
    this->B = Vector(Bb1, Bb2, Bb3);
    float Cb1 = this->b1.dot(W);
    float Cb2 = this->b2.dot(W);
    float Cb3 = this->b3.dot(W);
    this->C = Vector(Cb1, Cb2, Cb3);
}

float BTLocalization::rssiConv(float s)
{
    //Convert rssi to linear, positive = stronger format
    //Dummy for now
    return s;
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

void BTLocalization::calibrate(float r1, float r2, float r3) //Calibrate- solve calcRadius for k
{
    float s1, s2, s3 = 0;
    getStrs(&s1, &s2, &s3);
    if (s1 != 0)
    {
        this->k1 = r1 * s1 * s1;
    }
    if (s2 != 0)
    {
        this->k2 = r2 * s2 * s2;
    }
    if (s3 != 0)
    {
        this->k3 = r3 * s3 * s3;
    }
}

void BTLocalization::getStrs(float* s1, float* s2, float* s3) //Get strengths
{
    std::vector<btevent> events = this->manager->getResults(); //Get the result list
    for (btevent event : events) 
    {
        switch(event.r_id) 
        {
        case 1:
            *s1 = event.rssi;
            break;
        case 2:
            *s2 = event.rssi;
            break;
        case 3:
            *s3 = event.rssi;
            break;
        }
    }
}

void BTLocalization::getPos() 
{
    float s1, s2, s3 = 0;
    getStrs(&s1, &s2, &s3); //Get strengths
    getRadii(s1, s2, s3);
    float sx, sy, sz = 0;
    multilaterate(&sx, &sy, &sz, this->r1, this->r2, this->r3, sx, sy, sz, 0);
    Vector s(sx, sy, sz);
    //Find amount of each vector
    this->x = s.x * this->b1.x + s.y * this->b2.x + s.z * this->b3.x + this->bAnchor.x;
    this->y = s.x * this->b1.y + s.y * this->b2.y + s.z * this->b3.y + this->bAnchor.y;
    this->z = s.x * this->b1.z + s.y * this->b2.z + s.z * this->b3.z + this->bAnchor.z;
}
