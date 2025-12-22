#include "preload.h"

float rescale(float x, float i_min, float i_max, float o_min, float o_max)
{
    if(i_max - i_min == 0) return i_max;
    return ((o_max - o_min) * (x - i_min) / (i_max - i_min) + o_min);
}



