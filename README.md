# Motor Control System:

Based on the [Book](https://hades.mech.northwestern.edu/images/e/e3/EmbeddedComputingMechatronicsSampleChapters.pdf): Embedded Computing and Mechatronics with the PIC32 Microcontroller

System Block Diagram:
<p align="center">
  <img src="/images/dgm.png" alt="Alt text" width="700"/>
</p>

I-TEST Results:
<p align="center">
  <img src="/results/I-TEST.png" alt="Alt text" width="300"/>
</p>
 

## The system supports multiple commands:
- 'a': Reads the current counts from the encoder and sends it to the computer.
- 'b': Reads the current in mA and sends it to the computer.
- 'c': Reads the encoder count and sends it to the computer.
- 'd': Converts the encoder count to an angle and sends it to the computer.
- 'e': Sends a command to the encoder (details not specified).
- 'f': Sets the system to PWM mode and sets up the current control ISR.
- 'r': Retrieves and sends the current mode of the system.
- 'p': Powers down the motor and sets the system to IDLE mode.
- 'q': Sets the system to IDLE mode (quit).
- 'g': Sets the current control gains (Kp and Ki).
- 'h': Reads and sends the current control gains.
- 'k': Sets the system to ITEST mode and sets up the current control ISR.
- 'i': Sets the position control gains (Kp, Ki, and Kd).
- 'j': Reads and sends the position control gains.
- 'l': Sets a desired angle for the motor and switches to HOLD mode.
- 'm': Reads and sets a step trajectory for the motor.
- 'n': Reads and sets a cubic trajectory for the motor.
- 'o': Starts trajectory tracking mode.
- 'q': Quit the menu.
  
