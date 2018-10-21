#!/usr/bin/env python
# Import the relevant libraries
import RPi.GPIO as GPIO
import time

LedPin = 11                                     # pin for LED
BtnPin = 15                                     # pin connected to button
lastLEDstate = False                            # LED is intially off

def setup():
        ''' One time set up configurations'''
        GPIO.setmode(GPIO.BOARD)                # Numbers GPIOs by physical location
        GPIO.setup(LedPin, GPIO.OUT)            # Set LedPin's mode is output
        GPIO.setup(BtnPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)    # Set BtnPin's mode is input, and pull up to high level (3.3V)

        GPIO.output(LedPin, GPIO.LOW)          # Set LedPin low to turn the led off 

def toggleLED():
        global lastLEDstate
        if lastLEDstate == False:
           GPIO.output(LedPin, GPIO.HIGH)      	# Turn on
           lastLEDstate = True
        else:
           GPIO.output(LedPin, GPIO.LOW)       	# Turn off
           lastLEDstate = False


           
def loop():
        state = 0
        while True:
                # This code repeats forever
                if (state == 0 and  GPIO.input(BtnPin)== 0):
                   print("button was pressed")
                   state = 1
                if (state == 1 and  GPIO.input(BtnPin)== 1):
                   print("button was released")
                   state = 0
                   toggleLED()
                   
def destroy():
        GPIO.output(LedPin, GPIO.LOW)          	# Led off
        GPIO.cleanup()                          # Release resource


if __name__ == '__main__':                      # Program starts here
        setup()
        try:
                loop()
        except KeyboardInterrupt:               # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
                destroy()

