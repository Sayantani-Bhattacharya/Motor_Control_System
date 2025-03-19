#include "positionControl.h"

#define MAX_PTEST_COUNTER 99
#define PLOTPTS 100   // number of data points to plot
#define BUF_SIZE 200
#define MAX_ARRAY_SIZE 1000


volatile float kp_pc = 1.0, ki_pc = 0.0, kd_pc = 0.0;  
static int ptest_counter = 0;
static int ref_counter = 0;
static volatile float desiredAngle = 0.0;
static volatile float commandedCurrent = 0.0;
static float refAngle[PLOTPTS];
static float actAngle[PLOTPTS];
static float refTraj[MAX_ARRAY_SIZE];


void __ISR(_TIMER_4_VECTOR, IPL6SOFT) Position_Controller(void)
{
    static int counter = 0;
    static float eint = 0;
    static float eprev = 0;
    int operatingMode = get_mode(); 

    if (operatingMode == 3) // HOLD
    {
        WriteUART2("a");
		while (!get_encoder_flag())
		{
		}
		set_encoder_flag(0);
		int cnt = get_encoder_count();
		float degreesPerCount = 360.0 / (334 * 4);
		float encoderAngle = cnt * degreesPerCount;
		
		float e = desiredAngle - encoderAngle;
		float edot = e - eprev;
		eint = eint + e;
		commandedCurrent = (kp_pc*e) + (ki_pc*eint) + (kd_pc*edot);
		eprev = e;
		actAngle[ptest_counter] = encoderAngle;
		refAngle[ptest_counter] = desiredAngle;
		ptest_counter++;
	}

	else if(operatingMode == 0)  // IDLE
    {
		ptest_counter = 0;
		eint = 0.0;
		eprev = 0.0;
	}

    else if (operatingMode == 4) // TRACK
    {
        WriteUART2("a");
        while (!get_encoder_flag())
        {
        }
        set_encoder_flag(0);
        int cnt = get_encoder_count();
        float degreesPerCount = 360.0 / (334 * 4);
        float encoderAngle = cnt * degreesPerCount;

        float e = refTraj[ref_counter] - encoderAngle;
        float edot = e - eprev;
        eint = eint + e;
        commandedCurrent = (kp_pc * e) + (ki_pc * eint) + (kd_pc * edot);
        eprev = e;
        actAngle[ptest_counter] = encoderAngle;
        refAngle[ptest_counter] = refTraj[ref_counter];
        ptest_counter++;
    }

    IFS0bits.T4IF = 0; // clear interrupt flag IFS0<8>
}

float get_commanded_current(void)
{
    return commandedCurrent;
}

void set_desired_trajectory(int refTraj[])
{
    // Take this as user input
    refTraj = refTraj;
}

void set_desired_angle(float angle)
{
    // Take this as user input
    desiredAngle = angle;
}

void position_ISR_Setup(void)
{
    __builtin_disable_interrupts(); // INT step 2: disable interrupts at CPU
    PR4 = 7500 - 1; // 200Hz ISR
    TMR4 = 0; // INT step 3: clear the timer
    T4CONbits.TCKPS = 0; // Timer4 prescaler N=1 (1:1)  
    IPC4bits.T4IP = 6; // INT step 4: priority
	IPC4bits.T4IS = 0; // subpriority
	IFS0bits.T4IF = 0; // INT step 5: clear interrupt flag
	IEC0bits.T4IE = 1;  // INT step 6: enable interrupt
	T4CONbits.ON = 1; // INT step 7: enable the timer
    __builtin_enable_interrupts(); // INT step 7: enable interrupts at CPU
}

void set_position_gains(float kp, float ki, float kd)
{
    kp_pc = kp;
    ki_pc = ki;
    kd_pc = kd;
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