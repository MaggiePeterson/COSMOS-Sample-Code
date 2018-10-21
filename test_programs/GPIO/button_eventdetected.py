# This code illustrates the use of "add_event_detect"
# This is a non-blocking statement that is used in a loop to make sure you didn't miss an event

#!/usr/bin/env python
# Import the relevant libraries
import RPi.GPIO as GPIO
import time

LedPin = 11                                     # pin for LED
BtnPin = 15                                     # pin connected to button
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

                # Add event detection to a pin. This statement is non-blocking.
                # The subsequent code is executed (in this case, the LED is blinking).
                # If during any of this time, the button is pressed, this will be remembered. Afterward, the
                # 'event_detected' code is executed accordingly.
                # Possible options are: GPIO.RISING, GPIO.FALLING or GPIO.BOTH
                GPIO.add_event_detect(BtnPin, GPIO.FALLING)

                print('Start checking')
                
                for i in range(50):
                        toggleLED()
                        time.sleep(0.1)

                if GPIO.event_detected(BtnPin):
                        print('Button was pressed')
                        toggleLED()
                else:
                        print('Button was not pressed')

			# Disable event handling
                GPIO.remove_event_detect(BtnPin)
                
                time.sleep(2)

    
                   
def destroy():
        GPIO.output(LedPin, GPIO.LOW)          	# Led off
        GPIO.cleanup()                          # Release resource


if __name__ == '__main__':                      # Program starts here
        setup()
        try:
                loop()
        except KeyboardInterrupt:               # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
                destroy()

