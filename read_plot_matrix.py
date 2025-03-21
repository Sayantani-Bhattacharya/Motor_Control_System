# sudo apt-get install python3-pip
# python3 -m pip install pyserial
# sudo apt-get install python3-matplotlib

from genref import genRef
import matplotlib.pyplot as plt 
from statistics import mean 
import serial
ser = serial.Serial('/dev/ttyUSB0',230400)

print('Opening port: ')
print(ser.name)

def read_plot_matrix():
    n_str = ser.read_until(b'\n');  # get the number of data points to receive
    n_int = int(n_str) # turn it into an int
    print('Data length = ' + str(n_int))
    ref = []
    data = []
    data_received = 0
    while data_received < n_int:
        dat_str = ser.read_until(b'\n');  # get the data as a string, ints seperated by spaces
        dat_f = list(map(float,dat_str.split())) # now the data is a list
        ref.append(dat_f[0])
        data.append(dat_f[1])
        data_received = data_received + 1
    meanzip = zip(ref,data)
    meanlist = []
    for i,j in meanzip:
        meanlist.append(abs(i-j))
    score = mean(meanlist)
    t = range(len(ref)) # index array
    plt.plot(t,ref,'r*-',t,data,'b*-')
    plt.title('Score = ' + str(score))
    plt.ylabel('value')
    plt.xlabel('index')
    plt.show()

has_quit = False
# Menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # Display the menu options.
    print('\ta: read current sensor (ADC counts) \tb: read current sensor (mA) \tc: get encoder counts \td: read encoder angle \te: reset /'
    'encoder  \tf: Set PWM (-100 to 100) \tg: Set current gains \th: Get current gains \tk: Test current control  \to: Read plot matrix /'
    '\tm: Load step trajectory   \tn: Load cubic trajectory  \to: Execute trajectory   /'
    '\tp: Unpower the motor \tr: read the mode \ti:Set position gains \tj:Get position gains \tl:Go to angle (deg) \tq: Quit') # '\t' is a tab
    
    # Read the user's choice
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection+'\n'
     
    # Send the command to the PIC32
    ser.write(selection_endline.encode()); # .encode() turns the string into a char array
    
    if (selection == 'a'):
        print('Requesting current sensor (ADC counts): ')
        n_str = ser.read_until(b'\n'); 
        n_int = float(n_str)
        print('current sensor (ADC counts) value = '+str(n_int)+'\n')
    
    elif (selection == 'b'):
        print('Requesting current sensor (mA): ')
        n_str = ser.read_until(b'\n'); 
        n_int = float(n_str)
        print('current sensor (mA) value = '+str(n_int)+'\n')
    
    elif (selection == 'c'):
        print('Requesting encoder counts: ')
        n_str = ser.read_until(b'\n'); 
        n_int = int(n_str)
        print('Encoder counts = '+str(n_int)+'\n')
    
    elif (selection == 'd'):
        print('Requesting encoder angle: ')
        n_str = ser.read_until(b'\n'); 
        n_int = float(n_str)
        print('Encoder angle = '+str(n_int)+'\n')
    
    elif (selection == 'e'):
        print('Resetting encoder: ')
        
    elif (selection == 'f'):
        print('Setting PWM (-100 to 100): ')  
        pwm_input = input("Enter PWM value (-100 to 100): ")
        pwm_int = int(pwm_input)
        print(f"PWM value set to {pwm_int}\n")
        ser.write((str(pwm_int)+'\n').encode())

    elif (selection == 'g'):
        # Convert into float and send to PIC
        kp_n_str = input('Enter current gain kp: ')
        ki_n_str = input('Enter current gain ki: ') 
        kp = float(kp_n_str) # turn it into an int
        ki = float(ki_n_str)
        print(f"Current gains set to Kp: {kp} and Ki: {ki}\n")  # print it to the screen to double check
        current_gain_str = f"{kp} {ki}\n" # send the number
        print(f"Current gains string: {current_gain_str}")
        ser.write(current_gain_str.encode())

    elif (selection == 'h'):
        print('Get Current gains: ')
        gains = ser.readline().decode().strip()
        kp_value = float(gains.split("Kp:")[1].split(",")[0].strip())
        ki_value = float(gains.split("Ki:")[1].strip())
        print(f"Kp: {kp_value} and Ki: {ki_value}\n")

    elif (selection == 'i'):
        Kpos_input = input("Enter proportional gain value: ")
        Kipos_input = input("Enter integral gain value: ")
        Kdpos_input = input("Enter derivative gain value: ")
        Kpos_flt = float(Kpos_input)
        Kipos_flt = float(Kipos_input)
        Kdpos_flt = float(Kdpos_input)
        print(f"Position gains set to Kp: {Kpos_flt}, Ki: {Kipos_flt}, Kd: {Kdpos_flt}\n")
        gains_str = f"{Kpos_flt} {Kipos_flt} {Kdpos_flt}\n"
        ser.write(gains_str.encode())
    
    elif (selection == 'j'):
        gains = ser.readline().decode().strip()
        kp_value = float(gains.split("Kp:")[1].split(",")[0].strip())
        ki_value = float(gains.split("Ki:")[1].split(",")[0].strip())
        kd_value = float(gains.split("Kd:")[1].strip())
        print(f"Kp: {kp_value}, Ki: {ki_value} and Kd: {kd_value}\n")
   
    elif (selection == 'l'):
        desiredInput= input("Entire the desired angle in degrees: ")
        desiredAngle = float(desiredInput)
        print(f"Desired angle set to: {desiredAngle}\n")
        angle_str = f"{desiredAngle}\n"
        ser.write(angle_str.encode())
        read_plot_matrix()

    elif (selection == 'k'):
        print('Testing current control gains:\n ')
        read_plot_matrix()

    elif (selection == 'p'):
        print('Unpower the motor: ')   

    elif (selection == 'r'):
        print('Requesting mode: ')
        n_str = ser.read_until(b'\n'); 
        n_int = int(n_str)
        print('Mode = '+str(n_int)+'\n')

    elif(selection == 'm'):
        ref = genRef('step')
        size = len(ref)
        print(size)
        t = range(len(ref))
        plt.plot(t, ref, 'r*-')
        plt.ylabel('value')
        plt.xlabel('index')
        plt.show()
        ser.write(f"{size}\n".encode())
        for val in ref:
            ser.write(f"{val}\n".encode())

    elif (selection == 'n'):
        cref = genRef('cubic')
        cubicsize = len(cref)
        print(cubicsize)
        t = range(len(cref))
        plt.plot(t, cref, 'r*-')
        plt.ylabel('value')
        plt.xlabel('index')
        plt.show()
        ser.write(f"{cubicsize}\n".encode())
        for val in cref:
            ser.write(f"{val}\n".encode())

    elif (selection == 'o'):
        print(f"Executing Trajectory!\n")
        read_plot_matrix()


    elif (selection == 'o'):
        read_plot_matrix()
        
    elif (selection == 'q'):
        print('Exiting client')
        has_quit = True; # exit client
        # Close the port
        ser.close()
    else:
        print('Invalid Selection ' + selection_endline)
