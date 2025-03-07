#include "nu32dip.h"           // config bits, constants, funcs for startup and UART
// include other header files here
#include "encoder.h"
#define BUF_SIZE 200

int main() 
{
  char buffer[BUF_SIZE];
  NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  UART2_Startup();      
  __builtin_disable_interrupts();
  // in future, initialize modules or peripherals here
  __builtin_enable_interrupts();

  while(1)
  {
    NU32DIP_ReadUART1(buffer,BUF_SIZE); // we expect the next character to be a menu command
    // NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
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
        // int n = 0;        // dummy command for demonstration purposes
        // NU32DIP_ReadUART1(buffer,BUF_SIZE);
        // sscanf(buffer, "%d", &n);
        // sprintf(buffer,"%d\r\n", n + 1); // return the number + 1
        // NU32DIP_WriteUART1(buffer);
        // break;
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
      case 'e':
      {
        // read the encoder count and send it to the computer
        WriteUART2("b");
        break;
      }
      case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here. 
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
