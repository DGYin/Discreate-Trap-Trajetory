#ifndef ALG_COMPARATOR_H
#define ALG_COMPARATOR_H

#include <stdint.h>

float AlgComparator_Saturation(float input, float min, float max);
int32_t AlgComparator_SaturationInt(int32_t input, int32_t min, int32_t max);

#endif
