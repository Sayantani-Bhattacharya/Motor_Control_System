#include "positionControl.h"

volatile float kp_pc = 1.0, ki_pc = 10.0, kd_pc = 0.0;               // PI gains


void set_position_gains(float kp, float ki, float kd)
{
    kp_pc = kp;
    ki_pc = ki;
    kd_pc = kd;
    // kp_cc = 9.0;
}

float getKp_position(void)
{
    return kp_pc;
}

float getKi_position(void)
{
    return ki_pc;
}

float getKd_position(void)
{
    return kd_pc;
}   