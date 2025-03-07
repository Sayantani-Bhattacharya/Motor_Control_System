#include "isense.h"

void ADC_Startup(){
    INA219_Startup();
}

float get_current_counts()
{
    // Consider reading the ADC a few times and averaging for a more stable reading.
    // count (0-1023)
    float a = INA219_read_current();
    return a;
}

float get_current_mA()
{
    float a = readINA219(0x04);
    return a;    
}