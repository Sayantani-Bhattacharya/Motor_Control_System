#include "currentControl.h"
#include "ina219.h"

// #include "main.c"

#define NUMSAMPS 1000 // number of points in waveform
#define PLOTPTS 100   // number of data points to plot
#define DECIMATION 10 // plot every DECIMATIONth sample plot every 10th data
#define MAX_ITEST_COUNTER 99
#define MAX_HOLD_COUNTER 1000
#define BUF_SIZE 200

// ADC information
#define MAXPLOTPTS 1000
#define SAMPLE_TIME 6 // 24MHz*250ns

static volatile int Waveform[NUMSAMPS]; // waveform
static float REFCurrent[PLOTPTS]; // reference values to plotx
static float ACTCurrent[PLOTPTS]; // measured values to plot
volatile float dutyCycle = 0;
static volatile float desiredAngle = 0.0;
static float refAngle[MAX_HOLD_COUNTER];
static float actAngle[MAX_HOLD_COUNTER];

volatile float kp_cc = 1.5, ki_cc = 0.1; // PI gains
static int itest_counter = 0; // counter for ITEST mode
static int hold_counter = 0;  // counter for HOLD mode
static float eint = 0;        // integral error for current control
// volatile int dutyCycle = 0;


void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Current_Controller(void)
{
    static int counter = 0;
    int operatingMode = get_mode(); // get the current mode
    // IDLE MODE
    if (operatingMode == 0)
    {
        OC1RS = 0; // turn off the PWM output, break the motor motion.

        return;
    }
    // PWM MODE
    else if (operatingMode == 1)
    {
        if (dutyCycle > 0)
        {
            OC1RS = PR3 * dutyCycle / 100;
            set_direction(1); // set the direction to forward
        }
        else if (dutyCycle < 0)
        {
            OC1RS = -PR3 * dutyCycle / 100;
            set_direction(-1); // set the direction to forward
        }
        return;
    }
    // ITEST MODE
    else if (operatingMode == 2)
    {
        static int Val = 200;
        static float eint = 0;

        // char buffer[BUF_SIZE];
        // sprintf(buffer, "Current: %f\n\r", INA219_read_current());
        // NU32DIP_WriteUART1(buffer);

        // OC1RS =  Val;
        // set_direction(1);   // set the direction to forward

        float actCurrent = INA219_read_current(); // read the ADC value

        if (itest_counter == 25)
        {
            // set_direction(-1);
            Val = -200;
            // float error = - actCurrent - Val;
        }
        else if (itest_counter == 50)
        {
            Val = 200;
            // set_direction(1);
        }
        else if (itest_counter == 75)
        {
            // set_direction(-1);
            Val = -200;
        }
        else if (itest_counter == MAX_ITEST_COUNTER)
        {
            set_mode(0);       // set the mode to IDLE
            itest_counter = 0; // roll the counter over when needed
            eint = 0;          // reset the integral error
            perform_i_test();
        }
        // Add one to counter every time ISR is entered.
        itest_counter += 1;
        operatingMode = get_mode();

        float error = Val - INA219_read_current(); // error = reference - actual
        eint = eint + error;                       // accumulate the error

        dutyCycle = (kp_cc * error) + (ki_cc * eint);
        // write the duty cycle to uart

        

        if (dutyCycle > 100.0)
        {
            dutyCycle = 100.0;
        }
        else if (dutyCycle <= -100.0)
        {
            dutyCycle = -100.0;
        }


        OC1RS = (unsigned int)(abs(dutyCycle) / 100.0 * PR3);
        // if (dutyCycle != 0.0 )
        // {
        //     char buffer[BUF_SIZE];
        //     sprintf(buffer, " OC1RS: %d\n\r", OC1RS);
        //     NU32DIP_WriteUART1(buffer);
        // }
        if (dutyCycle >= 0)
        {
            set_direction(-1);
            // NU32DIP_WriteUART1("Forward\n\r");
        }
        else
        {
            set_direction(1);
            // NU32DIP_WriteUART1("Backward\n\r");
        }
        REFCurrent[itest_counter] = Val;
        ACTCurrent[itest_counter] = actCurrent;

    }    
    // HOLD MODE
    else if (operatingMode == 3)
    {
        float actCurrent = get_current_mA();
        float desiredCurrent = get_commanded_current();
        float error = desiredCurrent - actCurrent;
        eint = eint + error;
        dutyCycle = (kp_cc * error) + (ki_cc * eint);
        if (dutyCycle > 100.0)
        {
            dutyCycle = 100.0;
        }
        if (dutyCycle <= -100.0)
        {
            dutyCycle = -100.0;
        }
        OC1RS = (unsigned int)(abs(dutyCycle) / 100.0 * PR3);
        if (dutyCycle >= 0)
        {
            set_direction(-1);
            // NU32DIP_WriteUART1("Forward\n\r");
        }
        else
        {
            set_direction(1);
            // NU32DIP_WriteUART1("Backward\n\r");
        }
        
        // Reached Max counter value.
        hold_counter++;

        // Plot the data.
        int cnt = get_encoder_count();
		float degreesPerCount = 360.0 / (334 * 4);
		float encoderAngle = cnt * degreesPerCount;
        actAngle[hold_counter] = encoderAngle;
		refAngle[hold_counter] = desiredAngle;

        if (hold_counter >= MAX_HOLD_COUNTER)
        {
            set_mode(0);
            plot_current_data();
            counter = 0;
            eint = 0;
        }
    }
    // TRACK MODE
    else if (operatingMode ==4)
    {
        float actCurrent = get_current_mA();
        float desiredCurrent = get_commanded_current();
        float error = desiredCurrent - actCurrent;
        eint = eint + error;
        dutyCycle = (kp_cc * error) + (ki_cc * eint);
        if (dutyCycle > 100.0)
        {
            dutyCycle = 100.0;
        }
        if (dutyCycle <= -100.0)
        {
            dutyCycle = -100.0;
        }
        OC1RS = (unsigned int)(abs(dutyCycle) / 100.0 * PR3);
        if (dutyCycle >= 0)
        {
            set_direction(-1);
            // NU32DIP_WriteUART1("Forward\n\r");
        }
        else
        {
            set_direction(1);
            // NU32DIP_WriteUART1("Backward\n\r");
        }
        
        // Reached Max counter value.
        hold_counter++;

        // Plot the data.
        int cnt = get_encoder_count();
		float degreesPerCount = 360.0 / (334 * 4);
		float encoderAngle = cnt * degreesPerCount;
        actAngle[hold_counter] = encoderAngle;
		refAngle[hold_counter] = desiredAngle;

        if (hold_counter >= MAX_HOLD_COUNTER)
        {
            set_mode(0);
            plot_current_data();
            counter = 0;
            eint = 0;
        }
    }
    IFS0bits.T2IF = 0; // clear interrupt flag IFS0<8>
}

float set_desired_angle_in_cc(float angle)
{
    // Take this as user input
    desiredAngle = angle;
}


void plot_current_data(void)
{
    // Code to transfer the data of plots to python.
    char buffer[BUF_SIZE];
    int current_mode = get_mode();

    if (current_mode == 0)
    {
        sprintf(buffer, "%d\r\n", MAX_HOLD_COUNTER);
        NU32DIP_WriteUART1(buffer);
        for (int i = 0; i < MAX_HOLD_COUNTER; i++)
        {
            sprintf(buffer, "%f %f\r\n", refAngle[i], actAngle[i]);
            NU32DIP_WriteUART1(buffer);
        }
    }    
}


void perform_i_test(void)
{
    // Code to trandfer the data of plots to python.
    char buffer[BUF_SIZE];
    int current_mode = get_mode();
    // while (current_mode != 0)
    // {
    //     sprintf(buffer,"Current Mode: %d\n\r", current_mode);
    //     NU32DIP_WriteUART1(buffer);
    //     int current_mode = get_mode();
    // }
    if (current_mode == 0)
    {
        sprintf(buffer, "%d\r\n", PLOTPTS);
        NU32DIP_WriteUART1(buffer);
        for (int i = 0; i < PLOTPTS; i++)
        {
            sprintf(buffer, "%f %f\r\n", REFCurrent[i], ACTCurrent[i]);
            NU32DIP_WriteUART1(buffer);
        }
    }
}

void makeWaveform()
{
    // int i = 0, center = (PR2+1)/2, A = (PR2 +1)/4 ; // square wave, fill in center value and amplitude
    int i = 0, center = 1500, A = 1400;
    for (i = 0; i < NUMSAMPS; ++i)
    {
        if (i < NUMSAMPS / 2)
        {
            Waveform[i] = center + A;
        }
        else
        {
            Waveform[i] = center - A;
        }
    }
}

void set_direction(int dir)
{
    TRISACLR = 0x1; // Set RA0 as output
    if (dir == 1)
    {

        LATASET = 0x1; // Set RA0 HIGH: Amti-clockwise.
    }
    else if (dir == -1)
    {
        LATACLR = 0x1; // Set RA0 LOW: Clockwise
    }
}

void current_ISR_Setup(void)
{
    __builtin_disable_interrupts(); // INT step 2: disable interrupts at CPU
    // char message[100]; // message to send to MATLAB
    // set RB15 (Pin:26) to OC1: remapping
    ANSELA = 0;
    RPB15Rbits.RPB15R = 0b0101;
    T2CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
    PR2 = 9600 - 1; // 5kHz ISR  set period register, period = (PR2+1) * N * 12.5 ns 
    TMR2 = 0;

    // Timer 3 for ISR frequency
    PR3 = 2399;             // 20kHz PWM frequency
    TMR3 = 0;               // clear timer 3
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults

    makeWaveform();
    T2CONbits.ON = 1;  // turn on Timer2
    OC1CONbits.ON = 1; // turn on OC1

    // Interrupts settings.
    IPC2bits.T2IP = 5;             // INT step 4: priority
    IPC2bits.T2IS = 0;             //             subpriority
    IFS0bits.T2IF = 0;             // INT step 5: clear interrupt flag
    IEC0bits.T2IE = 1;             // INT step 6: enable interrupt
    __builtin_enable_interrupts(); // INT step 7: enable interrupts at CPU
}

void set_dutycycle(int dc)
{
    dutyCycle = dc;
}

void set_gains(float kp, float ki)
{

    kp_cc = kp;
    ki_cc = ki;
    // kp_cc = 9.0;
}

float getKp_current(void)
{
    return kp_cc;
}

float getKi_current(void)
{
    return ki_cc;
}