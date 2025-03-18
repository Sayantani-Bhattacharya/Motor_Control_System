#include "nu32dip.h"           // config bits, constants, funcs for startup and UART
// #include "utilities.h"
#include "encoder.h"
#include "ina219.h"
#include "isense.h"
#include "currentControl.h"
#include "positionControl.h"


#define BUF_SIZE 200
static volatile float kp_pc, ki_pc, kd_pc = 0.0;


int main() 
{
  char buffer[BUF_SIZE];
  NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  UART2_Startup();      
  INA219_Startup(); // initialize the INA219 current sensor
  ADC_Startup();


  __builtin_disable_interrupts();
  // in future, initialize modules or peripherals here
  __builtin_enable_interrupts();

  while(1)
  {
    NU32DIP_ReadUART1(buffer,BUF_SIZE); // we expect the next character to be a menu command
    // NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      case 'a':
      {
        // read the encoder count and send it to the computer
        char m[50];
        float p = get_current_counts();
        sprintf(m, "%.2f\r\n", p);  // Format to 2 decimal places
        NU32DIP_WriteUART1(m);
        break;
      } 
      case 'b':
      {
        // read the encoder count and send it to the computer
        char m[50];
        float p = get_current_mA();
        sprintf(m, "%.2f\r\n", p);  // Format to 2 decimal places
        NU32DIP_WriteUART1(m);
        break;
      } 
      case 'c':
      {
        // read the encoder count and send it to the computer
        WriteUART2("a");
        while(!get_encoder_flag()) {;}
        set_encoder_flag(0);
        char m[50];
        int p = get_encoder_count();
        sprintf(m, "%d\r\n", p);
        NU32DIP_WriteUART1(m);        
        break;
      }
      case 'd':                      
      {
        // read the encoder count and send it to the computer
        WriteUART2("a");
        while(!get_encoder_flag()) {;}
        set_encoder_flag(0);
        char m[50];
        int p = get_encoder_count();
        float angle = (p / (343.0 * 4)) * 360.0;  // Use 343.0 to force floating-point division
        sprintf(m, "%.2f\r\n", angle);  // Format to 2 decimal places
        NU32DIP_WriteUART1(m);
        break;
      }
      case 'e':
      {
        // read the encoder count and send it to the computer
        WriteUART2("b");
        break;
      } 
      case 'f':
      {
        // Set PWM mode and signal to the H-Bridge Output compare.
        set_mode(PWM);
        // Seting the position control ISR.
        position_ISR_Setup();
        break;
      }
      case 'r':
      {
        // Get mode.
        char m[50];
        sprintf(m, "%d\r\n", get_mode());
        NU32DIP_WriteUART1(m);
        break;
      }
      case 'p':
      {
        // Unpower the motor, and set the mode to IDLE.
        set_mode(IDLE);
        // Seting the position control ISR.
        position_ISR_Setup();
        break;
      }
      case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here. 
        set_mode(IDLE);
        break;
      }
      case 'g':
      {
        float ki_tmp, kp_tmp;
        // Set the current control gains.
        NU32DIP_ReadUART1(buffer, BUF_SIZE); // read the next character        
        sscanf(buffer, "%f %f", &kp_tmp, &ki_tmp);
        // ki_tmp = 89.0;
        set_gains(kp_tmp, ki_tmp);
        break;
      }
      case 'h':
      {
        // read current gains
        float kp_tmp, ki_tmp ;
        // get_gains(&kp_tmp, &ki_tmp);
        kp_tmp =getKp_current();
        ki_tmp = getKi_current();
        sprintf(buffer, "Kp: %f, Ki: %f\r\n", kp_tmp, ki_tmp);
        NU32DIP_WriteUART1(buffer);        
        break;
      }
      
      case 'k':
      {
        // Testing the current control.
        set_mode(ITEST);
        // Seting the position control ISR.
        position_ISR_Setup();
        // perform_i_test();
        // int current_mode = get_mode();
        // while (current_mode != 0)
        // {
        //   int current_mode = get_mode();
        // }
        // sprintf(buffer, "%d\r\n", PLOTPTS);
        // NU32DIP_WriteUART1(buffer);
        // if (currentMode == 0)
        // {
        //   for (int i = 0; i < PLOTPTS; i++)
        //   {
        //     sprintf(buffer, "%f %f\r\n", refCurrent[i], actCurrent[i]);
        //     NU32DIP_WriteUART1(buffer);
        //   }
        // }
        break;
      }

      default:
      {
        // NU32DIP_LED2 = 0;  // turn on LED2 to indicate an error.
        break;
      }
    }
  }
  return 0;
}
