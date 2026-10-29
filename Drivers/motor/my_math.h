#pragma once

#include "conf.h"

#define SQRT3 1.73205080756887729353f

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef clamp
#define clamp(x, low, high) (min(max((x), (low)), (high)))
#endif

#define MY_PI 3.14159265358979f
#define MY_TWO_PI 6.283185307179586f

/* 将角度快速归一化到 0~2pi。要求两次调用间角度变化不超过 2pi。 */
static inline float wrap_0_2pi_fast(float angle)
{
    if (angle >= MY_TWO_PI)
    {
        angle -= MY_TWO_PI;
    }
    else if (angle < 0.0f)
    {
        angle += MY_TWO_PI;
    }
    return angle;
}

/* 弧度 -> 正弦表索引（0~sin_table_size-1） */
static inline int rad_to_index(float angle)
{
    int index = (int)(angle * (float)sin_table_size / MY_TWO_PI);
    return index & (sin_table_size - 1);
}

/* 正弦表索引 -> 弧度 */
static inline float index_to_rad(int index)
{
    return (float)index * MY_TWO_PI / (float)sin_table_size;
}
