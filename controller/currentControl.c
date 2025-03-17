#include "currentControl.h"
// #include "main.c"

#define NUMSAMPS 1000     // number of points in waveform
#define PLOTPTS 100      // number of data points to plot
#define DECIMATION 10    // plot every DECIMATIONth sample plot every 10th data
#define MAX_ITEST_COUNTER 99
#define BUF_SIZE 200

// ADC information
#define MAXPLOTPTS 1000
#define SAMPLE_TIME 6 // 24MHz*250ns

static volatile int Waveform[NUMSAMPS];                  // waveform
static volatile int ADCarray[PLOTPTS];                   
static volatile int REFarray[PLOTPTS];                   

static float REFCurrent[PLOTPTS];   // reference values to plotx
static float ACTCurrent[PLOTPTS];   // measured values to plot
volatile int dutyCycle = 0;

volatile float kp_cc = 01.0, ki_cc = 10.0; // PI gains


// static volatile int StoringData = 0;                     // flag to start storing data if flag 1, currently storing.
static volatile int itest_counter = 0; // counter for ITEST mode

// static volatile float Kp=0, Ki=0, Kd=0, Eintmax=0;                  // PID gains
// static volatile int Eint = 0;                             // control effort integral

// unsigned int adc_sample_convert(int pin);
// Need to know if in position control mode, // ISR  // use getMode
// Case statements:  if in position control mode, hold, track, ....

// Short no of cases.

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Position_Controller(void) { // _TIMER_2_VECTOR = 8 
    static int counter = 0; 
    int operatingMode = get_mode(); // get the current mode
    // IDLE MODE
    if (operatingMode == 0) {
        OC1RS = 0; // turn off the PWM output, break the motor motion.
        return;
    } 
    // PWM MODE
    else if (operatingMode == 1) {
        // OC1RS = 3000;
        set_direction(1); // set the direction to forward
        OC1RS = Waveform[counter];              // OCIRS copies value to ocir at the right time, so its safer to set ocirs.         
        // Add one to counter every time ISR is entered
        if (counter == NUMSAMPS) {
            counter = 0; // roll the counter over when needed        
        }
    }    
    // ITEST MODE
    else if (operatingMode == 2) {
        static int Val = 1500;
        static float eint = 0;

        // OC1RS =  Val;
        set_direction(1);   // set the direction to forward

        float actCurrent = get_current_counts(); // read the ADC value

        if(itest_counter == 25)
        {
            set_direction(-1);
            Val = -1500;
            // float error = - actCurrent - Val;
        }
        else if (itest_counter == 50)
        {
            Val = 1500;
            set_direction(1);
        }
        else if (itest_counter == 75)
        {
            set_direction(-1);
            Val = -1500;
            // float error = - actCurrent - Val;
        }           
        else if (itest_counter == MAX_ITEST_COUNTER) {
            set_mode(0); // set the mode to IDLE 
            itest_counter = 0; // roll the counter over when needed
            eint = 0; // reset the integral error      
        }

        float error = Val - actCurrent; // error = reference - actual
        eint = eint + error; // accumulate the error

        dutyCycle = (kp_cc * error) + (ki_cc * eint);

		if (dutyCycle > 100.0)
		{
			dutyCycle = 100.0;
		}
		if (dutyCycle <= -100.0)
		{
			dutyCycle = -100.0;
		}
		OC1RS = (unsigned int)(abs(dutyCycle)/100.0 * PR3);
		LATAbits.LATA0 = (dutyCycle >= 0) ? 0 : 1;
        REFCurrent[itest_counter] = Val;
		ACTCurrent[itest_counter] = actCurrent;
        perform_i_test();

        // OC1RS =  Waveform[counter]; // turn off the PWM output, break the motor motion.        
        // Add one to counter every time ISR is entered.
        itest_counter += 1; 
    }
    IFS0bits.T2IF = 0;            // clear interrupt flag IFS0<8>
}

void perform_i_test(void)
{
    // Code to trandfer the data of plots to python.
    char buffer[BUF_SIZE];
    int current_mode = get_mode();
    while (current_mode != 0)
    {
        int current_mode = get_mode();
    }
    sprintf(buffer, "%d\r\n", PLOTPTS);
    NU32DIP_WriteUART1(buffer);
    if (current_mode == 0)
    {
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
        if ( i < NUMSAMPS/2) 
        {
            Waveform[i] = center + A;
        } 
        else 
        {
            Waveform[i] = center - A;
        }
    }
}

// void ADC_Startup(){
//   ANSELAbits.ANSA1 = 1; // AN1 is an adc pin
//   AD1CON3bits.ADCS = 1; // ADC clock period is Tad = 2*(ADCS+1)*Tpb =2*2*(1/48000000Hz) = 83ns > 75ns
//   AD1CON1bits.ADON = 1;
// }

// unsigned int adc_sample_convert(int pin)
// {
//   unsigned int elapsed = 0, finish_time = 0;
//   AD1CHSbits.CH0SA = pin;
//   AD1CON1bits.SAMP = 1;
//   elapsed = _CP0_GET_COUNT();
//   finish_time = elapsed + SAMPLE_TIME;
//   while (_CP0_GET_COUNT() < finish_time)
//   {
//     ;
//   }
//   AD1CON1bits.SAMP = 0;
//   while (!AD1CON1bits.DONE)
//   {
//     ;
//   }
//   return ADC1BUF0;
// }

void set_direction(int dir)
{
    TRISACLR = 0x1;  // Set RA0 as output
    if (dir == 1) {
        
        LATASET = 0x1;  // Set RA0 HIGH: Amti-clockwise.
        
    } 
    else if (dir == -1) {
        LATACLR = 0x1;  // Set RA0 LOW: Clockwise
    }
}

void position_ISR_Setup(void)
{
    // ADC_Startup();    
    __builtin_disable_interrupts(); // INT step 2: disable interrupts at CPU   
    // char message[100]; // message to send to MATLAB        
    // set RB15 (Pin:26) to OC1: remapping
    ANSELA = 0;
    RPB15Rbits.RPB15R = 0b0101;
    T2CONbits.TCKPS = 0;       // Timer2 prescaler N=1 (1:1)
    
    // int operatingMode = get_mode(); // get the current mode
    // if (operatingMode == 0) { //IDLE MODE
        
    //     return;
    // } 
    // else if (operatingMode == 1) {
    //     OC1RS = 0; // turn off the PWM output, break the motor motion.
    //     return;
    // } 
    // else if (operatingMode == 2) {
    //     OC1RS = 0; // turn off the PWM output, break the motor motion.
    //     return;
    // }
    // // Timer 2 for setting PWM waveform.
    // if (operating)
    PR2 = 3999;                // set period register, period = (PR2+1) * N * 12.5 ns = 20 kHz
    TMR2 = 0;






    // Timer 3 for ISR frequency
    PR3 = 3999;
    TMR3 = 0;                // clear timer 3
    OC1CONbits.OCM = 0b110;    // PWM mode without fault pin; other OC1CON bits are defaults
    
    makeWaveform();    
    T2CONbits.ON = 1;      // turn on Timer2
    OC1CONbits.ON = 1;     // turn on OC1

    // Interrupts settings. 
    IPC2bits.T2IP = 5;              // INT step 4: priority
    IPC2bits.T2IS = 0;              //             subpriority
    IFS0bits.T2IF = 0;              // INT step 5: clear interrupt flag
    IEC0bits.T2IE = 1;              // INT step 6: enable interrupt
    __builtin_enable_interrupts();  // INT step 7: enable interrupts at CPU
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