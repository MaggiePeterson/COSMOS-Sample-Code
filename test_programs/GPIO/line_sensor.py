#!/usr/bin/env python
# Import the relevant libraries
import RPi.GPIO as GPIO
import time

lnsensorPin = 13                                # pin 13

def setup():
        ''' One time set up configurations'''
        GPIO.setmode(GPIO.BOARD)                # Numbers GPIOs by physical location
        GPIO.setup(lnsensorPin,GPIO.IN)         # Set to input mode

def loop():
        while True:
                # This code repeats forever
                data = GPIO.input(lnsensorPin)  # Read from the pin
                print(data)
                time.sleep(0.2)


def destroy():
        GPIO.cleanup()                          # Release resource


if __name__ == '__main__':                      # Program starts here
        setup()
        try:
                loop()
        except KeyboardInterrupt:               # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
                destroy()



