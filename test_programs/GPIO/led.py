#!/usr/bin/env python
# Import the relevant libraries
import RPi.GPIO as GPIO
import time

LedPin = 11                                     # pin11

def setup():
        ''' One time set up configurations'''
        GPIO.setmode(GPIO.BOARD)                # Numbers GPIOs by physical location
                                                # An alternative is GPIO.BCM
        GPIO.setup(LedPin, GPIO.OUT)            # Set LedPin's mode is output
        GPIO.output(LedPin, GPIO.LOW)          	# Set LedPin low to turn the led off

def loop():
        while True:
                # This code repeats forever
                print('led on')
                GPIO.output(LedPin, GPIO.HIGH)  # led on
                time.sleep(0.5)
                print('led off')
                GPIO.output(LedPin, GPIO.LOW)  	# led off
                time.sleep(0.5)

def destroy():
        GPIO.output(LedPin, GPIO.LOW)          	# led off
        GPIO.cleanup()                          # Release resource

if __name__ == '__main__':                      # Program starts here
        setup()
        try:
                loop()
        except KeyboardInterrupt:               # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
                destroy()
