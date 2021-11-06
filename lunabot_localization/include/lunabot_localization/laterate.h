#pragma once

#include <cmath>

unsigned char multilaterate(float *_x, float *_y, float *_z, float r1, float r2, float r3, float x, float y, float z, unsigned char mode); 

unsigned char bilaterate_r3(float *_x, float *_y, float *_z, float r1, float r2, float z);
unsigned char bilaterate_r2(float *_x, float *_y, float *_z, float r1, float r3, float z); 
unsigned char bilaterate_r1(float *_x, float *_y, float *_z, float r2, float r3, float z);
unsigned char trilaterate(float *_x, float *_y, float *_z, float r1, float r2, float r3);

unsigned char test_range(float r, unsigned char rn, float x, float y, float z);
