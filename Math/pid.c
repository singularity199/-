#include "pid.h"
#include <stdlib.h>
#include <math.h>

#include "pid.h"

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

void PI_Reset(PI_t *pi)
{
    pi->integral = 0.0f;
}

float PI_Update(PI_t *pi, float err)
{
    pi->integral += err * pi->ki;
    pi->integral = clampf(pi->integral, -pi->integral_limit, pi->integral_limit);

    float out = err * pi->kp + pi->integral;
    out = clampf(out, -pi->out_limit, pi->out_limit);
    return out;
}
