#!/usr/bin/env python
# This is the basic code to do a serial interface

# Import the relevant libraries
import serial

# Initializes serial reading
ser = serial.Serial('/dev/ttyACM0',9600) 

# Execute forever
while True:

  # Checks if there is data to read
  if(ser.inWaiting()>0):  

    # Reads data and stores in myData variable
    myData=ser.readline() 
    print(myData)
