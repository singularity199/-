#ifndef __PID_H
#define __PID_H

typedef struct
{
    float kp;
    float ki;

    float integral;
    float integral_limit;

    float out_limit;
} PI_t;

void  PI_Reset(PI_t *pi);
float PI_Update(PI_t *pi, float err);

#endif
