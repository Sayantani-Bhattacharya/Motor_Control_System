#include "positionControl.h"


#define NUMSAMPS 1000     // number of points in waveform
#define PLOTPTS 200      // number of data points to plot
#define DECIMATION 10    // plot every DECIMATIONth sample plot every 10th data

// ADC information
#define MAXPLOTPTS 1000
#define SAMPLE_TIME 6 // 24MHz*250ns

static volatile int Waveform[NUMSAMPS];                  // waveform
static volatile int ADCarray[PLOTPTS];                   // measured values to plot
static volatile int REFarray[PLOTPTS];                   // reference values to plot
// static volatile int StoringData = 0;                     // flag to start storing data if flag 1, currently storing.

// static volatile float Kp=0, Ki=0, Kd=0, Eintmax=0;                  // PID gains
// static volatile int Eint = 0;                             // control effort integral


// unsigned int adc_sample_convert(int pin);


// Need to know if in position control mode, 
// ISR
// use getMode

// Case statements:  if in position control mode, hold, track, ....

// Short no of cases.

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Position_Controller(void) { // _TIMER_2_VECTOR = 8 
    static int counter = 0; 

    auto operatingMode = get_mode(); // get the current mode
    if (operatingMode == 0) {
        OC1RS = 0; // turn off the PWM output, break the motor motion.
        return;
    } 
    else if (operatingMode == 1) {
        // OC1RS = 3000;
        set_direction(1); // set the direction to forward
        OC1RS = Waveform[counter];              // OCIRS copies value to ocir at the right time, so its safer to set ocirs.         
        // Add one to counter every time ISR is entered
        if (counter == NUMSAMPS) {
            counter = 0; // roll the counter over when needed        
        }
    }
    
    IFS0bits.T2IF = 0;            // clear interrupt flag IFS0<8>

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
    // PR2 = 4800-1;           // period = (PR2+1) * N * 12.5 ns = 100 us, 10 kHz  || 12.5ns -> clock_period.
    PR2 = 3999;                // set period register, period = (PR2+1) * N * 12.5 ns = 20 kHz
    TMR2 = 0;
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

// int main_func(void){
//     NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
//     position_ISR_Setup(); // setup the position control ISR
//     while (1) 
//     {
//         ;
//         // NU32DIP_ReadUART1(message, sizeof(message)); // wait for a message from MATLAB
//         // sscanf(message, "%f %f %f" , &kptemp, &kitemp, &emax);


//         // __builtin_disable_interrupts(); // keep ISR disabled as briefly as possible
//         // Kp = kptemp; // copy local variables to globals used by ISR
//         // Ki = kitemp;
//         // Eint = 0; // reset the control effort integral
//         // Eintmax = emax;

//         // __builtin_enable_interrupts(); // only 2 simple C commands while ISRs disabled


//         // StoringData = 1; // message to ISR to start storing data
//         // while (StoringData) { // wait until ISR says data storing is done
//         // ; // do nothing 
//         // }
        
//         // For plotting data:
//         // for (i=0; i<PLOTPTS; i++) 
//         // { 
//         //     // send plot data to MATLAB
//         //     // when first number sent = 1, MATLAB knows we’re done
//         //     sprintf(message, "%d %d %d\r\n", PLOTPTS-i, ADCarray[i], REFarray[i]);
//         //     NU32DIP_WriteUART1(message);
//         // }
//     }
//     return 0; 
// }