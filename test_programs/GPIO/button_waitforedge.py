# This code illustrates the use of "wait_for_edge"
# This is a blocking statement that waits for an event on a pin.
# It is functionally equivalent to constant polling, but more efficient

#!/usr/bin/env python
# Import the relevant libraries
import RPi.GPIO as GPIO
import time

LedPin = 11                                     # pin11
BtnPin = 15                                     #pin connected to button
lastLEDstate = False                            # LED is intially off

def setup():
        GPIO.setmode(GPIO.BOARD)                # Numbers GPIOs by physical location
        GPIO.setup(LedPin, GPIO.OUT)            # Set LedPin's mode is output
        GPIO.setup(BtnPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)    # Set BtnPin's mode is input, and pull up to high level (3.3V)

        GPIO.output(LedPin, GPIO.LOW)           # Set LedPin low to turn the led off 

def toggleLED():
        global lastLEDstate
        if lastLEDstate == False:
           GPIO.output(LedPin, GPIO.HIGH)      	# Turn on
           lastLEDstate = True
        else:
           GPIO.output(LedPin, GPIO.LOW)       	# Turn off
           lastLEDstate = False


           
def loop():
        while True:

                # Wait for up to 5 seconds for a falling edge (timeout is in milliseconds).
                # This statement is blocking!
                # Possible options are: GPIO.RISING, GPIO.FALLING or GPIO.BOTH
                # This code is mainly to illustrate the use of 'timeout'.
                # If you don't need this part of the functionality but only the edge detection,
                # you can simply replace it by 'GPIO.wait_for_edge(BtnPin, GPIO_FALLING)'
                channel = GPIO.wait_for_edge(BtnPin, GPIO.FALLING, timeout=5000)
                if channel is None:
                        print('Timeout occurred')
                        for i in range(6):
                                toggleLED()
                                time.sleep(0.25)                       
                else:
                        print('Edge detected on channel', channel)
                        print("Button was pressed")
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

