#include <lunabot_localization/laterate.h>

//Error threshold - not currently working
//#define ETHRESH 0.00001


unsigned char bilaterate_r3(float UY, float UZ, float VY, float VZ, float WY, float WZ, float *_x, float *_y, float *_z, float r1, float r2, float z) 
{
    //Determines x and y with a known z and r1 and r2, but no r3
    //We then calculate y by subtracting the two sphere equations
    double c3 = z - UZ;
    double c6 = z - VZ;
    //y=(r1^2 +c6^2 - r2^2 - c3^2 - uy^2 +vy^2)/(2vy-2uy)
    double y = (r1 * r1 + c6 * c6 - r2 * r2 - c3 * c3 - UY * UY + VY * VY) / (2*VY - 2*UY);
    //Then just substitute in y to solve for x using the r1 equation
    double x2 = r1 * r1 - (y - UY) * (y - UY) - c3 * c3;
    if (x2 < 0) 
    {
        return 1;
    }
    else 
    {
        double x = std::sqrt(x2);
        *_x = (float) x;
        *_y = (float) y;
        return 0;
    }
}

unsigned char bilaterate_r2(float UY, float UZ, float VY, float VZ, float WY, float WZ, float *_x, float *_y, float *_z, float r1, float r3, float z) 
{
    //Determines x and y with a known z and r1 and r3, but no r2
    //We then calculate y by subtracting the two sphere equations
    double c3 = z - UZ;
    double c6 = z - WZ;
    //y=(r1^2 +c6^2 - r2^2 - c3^2 - uy^2 +vy^2)/(2vy-2uy)
    double y = (r1 * r1 + c6 * c6 - r3 * r3 - c3 * c3 - UY * UY + WY * WY) / (2*WY - 2*UY);
    //Then just substitute in y to solve for x using the r1 equation
    double x2 = r1 * r1 - (y - UY) * (y - UY) - c3 * c3;
    if (x2 < 0) 
    {
        return 1;
    }
    else 
    {
        double x = std::sqrt(x2);
        *_x = (float) x;
        *_y = (float) y;
        return 0;
    }
}

unsigned char bilaterate_r1(float UY, float UZ, float VY, float VZ, float WY, float WZ, float *_x, float *_y, float *_z, float r2, float r3, float z)
{
    //Determines x and y with a known z and r2 and r3, but no r1
    //We then calculate y by subtracting the two sphere equations
    double c3 = z - VZ;
    double c6 = z - WZ;
    //y=(r1^2 +c6^2 - r2^2 - c3^2 - uy^2 +vy^2)/(2vy-2uy)
    double y = (r3 * r3+ c6 * c6 - r2 * r2 - c3 * c3 - VY * VY + WY * WY) / (2*WY - 2*VY);
    //Then just substitute in y to solve for x using the r1 equation
    double x2 = r2 * r2 - (y - VY) * (y - VY) - c3 * c3;
    if (x2 < 0) 
    {
        return 1;
    }
    else 
    {
        double x = std::sqrt(x2);
        *_x = (float) x;
        *_y = (float) y;
        return 0;
    }
}


unsigned char trilaterate(float UY, float UZ, float VY, float VZ, float WY, float WZ, float *_x, float *_y, float *_z, float r1, float r2, float r3) 
{
    //See https://www.iri.upc.edu/files/scidoc/743-Revisiting-trilateration-for-robot-localization.pdf
    //By Thomas and Ros, 2005
    //We will first set up the system of linear equations (system 2 in the paper, equations 2 & 3)
    //For simplicity, the equations will become this form (we can ignore x because it is always at zero)
    //y * c1 + z * c2 = t1
    //y * c3 + z * c4 = t4
    //Where c1 = y2 - y1, c2 = z2 - z1, t1 = (r1^2 - r2^2 + a^2)/2
    double c1 = VY - UY;
    double c2 = VZ - UZ;
    double t1 = (r1 * r1 - r2 * r2 + ((VY-UY) * (VY - UY) + (VZ - UZ) * (VZ - UZ)))/2;
    //Second equation
    double c3 = WY - UY;
    double c4 = WZ - UZ;
    double t2 = (r1 * r1 - r2 * r2 + ((WY-UY) * (WY - UY) + (WZ - UZ) * (WZ - UZ)))/2;
    //We will then solve this using row reduction (on a virtual matrix)
    //The math here gets pretty gross
    //z becomes (t2 - (c3 * t1)/c1)/(c4 - (c2*c3/c1))
    double z = (t2 - (c3 * t1) / c1)/(c4 - (c2 * c3)/c1);
    //y becomes t1/c1 - c2/c1 * z
    //
    //TODO: Double check this on calculator
    //
    double y = t1 / c1 - (c2 / c1 * z);
    //Now that we have y and z, finding x becomes trivial
    //We only take the positive result because, given the arena, we know that x must be positive
    //First, we will do a sanity test
    double x2 =(r1*r1 - (y - UY) * (y - UY) - (z - UZ) * (z - UZ));
    //Test if this is negative- if it is, something has gone wrong
    if (x2 < 0) 
    {
        return 1;
    }
    else 
    {
        double x = std::sqrt(x2);
        *_x = (float) x;
        *_y = (float) y;
        *_z = (float) z;
        return 0;
    }
}

unsigned char test_range(float UY, float UZ, float VY, float VZ, float WY, float WZ, float r, unsigned char rn, float x, float y, float z) 
{
    double dy, dz;
    switch (rn)
    {
    case 1:
        dy = UY; dz = UZ;
        break;
    case 2:
        dy = VY; dz = VZ;
        break;
    case 3:
        dy = WY; dz = WZ;
        break;
    default:
        return 2;
        break;
    }
    return r*r == x*x + (y-dy) * (y-dy) + (z-dz) * (z-dz) ? 3 : 4;
}

unsigned char multilaterate(float UY, float UZ, float VY, float VZ, float WY, float WZ, float *_x, float *_y, float *_z, float r1, float r2, float r3, float x, float y, float z, unsigned char mode) 
{
    //Mode determines how bilateration works
    //0 - test values yourself
    //1 - ignore r1
    //2 - ignore r2
    //3 - ignore r3
    //4 - trilaterate
    //5 - test range r1
    //6 - test range r2
    //7 - test range r3
    if (mode == 0) //Test for errors if mode is default
    {
        //Calculate differences in radii
        double d12 = r1*r1 - r2*r2;
        double d13 = r1*r1 - r3*r3;
        double d23 = r2*r2 - r3*r3;
        //Compare with differences in expected position
        double e12 = (y - UY) * (y - UY) - (y - VY) * (y - VY) + (z - UZ) * (z - UZ) + (z - VZ) * (z - VZ) - d12;
        double e13 = (y - UY) * (y - UY) - (y - WY) * (y - WY) + (z - UZ) * (z - UZ) + (z - WZ) * (z - WZ) - d13;
        double e23 = (y - VY) * (y - VY) - (y - WY) * (y - WY) + (z - VZ) * (z - VZ) + (z - WZ) * (z - WZ) - d23;
        //Test if threshold crossed
        /*
        bool fail12 = e12 > ETHRESH || e12 < -ETHRESH;
        bool fail13 = e13 > ETHRESH || e13 < -ETHRESH;
        bool fail23 = e23 > ETHRESH || e23 < -ETHRESH;
        if (fail12 && fail13 && fail23) {mode = 5;}
        else 
        {
            if (fail12 && fail13) {mode = 1;}
            else if (fail12 && fail23) {mode = 2;}
            else if (fail13 && fail23) {mode = 3;}
            else 
            {
                mode = 4;
            }
        }
        */
       mode = 4; //until error checking works we will say mode is 4
    }
    unsigned char rn = 1;
    switch(mode) 
    {
        case 1:
            return bilaterate_r1(UY, UZ, VY, VZ, WY, WZ, _x, _y, _z, r2, r3, z);
            break;
        case 2:
            return bilaterate_r2(UY, UZ, VY, VZ, WY, WZ, _x, _y, _z, r1, r3, z);
            break;
        case 3:
            return bilaterate_r3(UY, UZ, VY, VZ, WY, WZ, _x, _y, _z, r1, r2, z);
            break;
        case 4:
            return trilaterate(UY, UZ, VY, VZ, WY, WZ, _x, _y, _z, r1, r2, r3);
            break;
        case 5:
            rn = 1;
            return test_range(UY, UZ, VY, VZ, WY, WZ, r1, rn, x, y, z);
            break;
        case 6:
            rn = 2;
            return test_range(UY, UZ, VY, VZ, WY, WZ, r2, rn, x, y, z);
            break;
        case 7:
            rn = 3;
            return test_range(UY, UZ, VY, VZ, WY, WZ, r3, rn, x, y, z);
            break;
        default:
            return 2;
            break;
    }
}
