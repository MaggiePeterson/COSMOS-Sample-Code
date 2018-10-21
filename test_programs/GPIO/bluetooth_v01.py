# In this example, an external Bluetooth module is used to communicate
# to a phone. The communication with the module is via the UART (serial
# communication). Note that the hardware serial needs to be enabled first in
# the RPi settings.
# This example reads multi-characted keywords.

# Libraries
import time
import serial

# Intialize the serial port
# If this doesn't work, change it to:
#  ser = serial.Serial('/dev/ttyACM0',9600) 
ser = serial.Serial("/dev/ttyS0", baudrate = 9600)


# Wait for this long for a character; else exit the getChar() function
# If you comment this out, the getChar() function will wait until a
# character has been received
ser.timeout = 0.1


# Process the characters from the serial bus
# After a valid word is detected, set the data = ''
data=''
def getChar(data):
    data = data+ser.read(1).decode()
    return data


print('Start program')

# Keep on reading:
while True:

    # Read a character and append it to the string data
    # Exit getChar() when a character is received or the timeout is reached
    data = getChar(data)

    word = ''
    # Check if the data is a valid word
    if (data == 'Up') or (data == 'Down'):
        word = data
        data=''

    # Act on the word received
    if (word == 'Up'):
        print('UP')
        ser.write(' Go up'.encode())

    elif (word == 'Down'):
        print('DOWN')
        ser.write(' Go down'.encode())    

    else:
        print('.')

# Close the serial
ser.close()
