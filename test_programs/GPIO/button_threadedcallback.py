# This code illustrates the use of "event_detect with threaded callback"
# This is essentially interrupt handling

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


def callbackFunction(channel):
        print('Edge detected on channel %s'%channel)
        print('Button was pressed')
        toggleLED()



 
           
def loop():
        while True:

                # Add event detection to a pin. This statement is non-blocking.
                # The subsequent code is executed (in this case, it is just a loop waiting).
                # If at any time, the button is pressed, the executing will jump to the callbackFunction and execute that code. Once the
                # callbackFunction has been executed, the program resumes where it was initially interrupted.
                # The 'bouncetime' handles switch bouncing (the time is in milliseconds).
                # Possible options are: GPIO.RISING, GPIO.FALLING or GPIO.BOTH
                GPIO.add_event_detect(BtnPin, GPIO.FALLING, callback = callbackFunction, bouncetime = 200)

                # This could be any code
                while True:
                        time.sleep(0.1)


    
                   
def destroy():
        GPIO.output(LedPin, GPIO.LOW)          	# Led off
        GPIO.cleanup()                          # Release resource


if __name__ == '__main__':                      # Program starts here
        setup()
        try:
                loop()
        except KeyboardInterrupt:               # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
                destroy()

