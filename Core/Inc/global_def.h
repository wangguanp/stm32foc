#pragma once
#ifndef  PI
#define PI 3.14159265358979
#endif

#define deg2rad(a) (PI * (a) / 180)
#define rad2deg(a) (180 * (a) / PI)
#ifndef max
#define max(a,b) ((a)>(b) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) ((a)<(b) ? (a) : (b))
#endif
