#include "Alg_Comparator.h"

float AlgComparator_Saturation(float input, float min, float max)
{
    if (input < min)
        return min;
    else if (input > max)
        return max;
    else
        return input;
}

int32_t AlgComparator_SaturationInt(int32_t input, int32_t min, int32_t max)
{
    if (input < min)
        return min;
    else if (input > max)
        return max;
    else
        return input;
}
