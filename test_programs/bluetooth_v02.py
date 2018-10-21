# In this example, an external Bluetooth module is used to communicate
# to a phone. The communication with the module is via the UART (serial
# communication). Note that the hardware serial needs to be enabled first in
# the RPi settings.
# This example reads one character at a time.

# Libraries
import time
import serial

# Intialize the serial port
ser = serial.Serial("/dev/ttyS0", baudrate = 9600)

# Wait for this long for a character; else exit the read() function
# If you comment this out, the read() function will wait until a
# character has been received
ser.timeout = 0.1




print('Start program')

# Keep on reading:
while True:

    # Read a single character
    # Exit read() when a character is received or the timeout is reached
    data = ser.read(1).decode()


    # Act on the character received
    if (data == 'U'):
        print('UP')
        ser.write(' Go up'.encode())

    elif (data == 'D'):
        print('DOWN')
        ser.write(' Go down'.encode())    

    else:
        print('.')

# Close the serial
ser.close()
