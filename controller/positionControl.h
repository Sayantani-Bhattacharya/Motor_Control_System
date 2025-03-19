#include "nu32dip.h"           // config bits, constants, funcs for startup and UART
#include "isense.h"
// #include "utilities.h"


// Function Declarations
void position_ISR_Setup(void);
float getKp_position(void);
float getKi_position(void);
float getKd_position(void);
void set_position_gains(float kp, float ki, float kd);
void set_desired_angle(float angle);
float get_commanded_current(void);
void set_desired_trajectory(float refTraj[]);








