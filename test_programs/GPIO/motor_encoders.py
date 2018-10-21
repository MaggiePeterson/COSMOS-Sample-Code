# Libraries
import RPi.GPIO as GPIO
import time
#import sys
 
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)
 
# Set GPIO Pins
# We are only using one of the motors in this example
GPIO_Ain1 = 11
GPIO_Ain2 = 13
GPIO_Apwm = 15
GPIO_Aencoder = 7


# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Aencoder, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# The PWM pin is set permanently to HIGH; this sets the motor speed to the maximum value
# If we want to change the motor speed, we need to attach a PWM signal to the pin
GPIO.output(GPIO_Apwm, True)

# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
motorA_state = 0


# Execute this function when there is an event on the encoder pin (i.e., a transition)
def encoderCount(channel):
    global encoderTicksA
    global encoderTargetA
    global motorA_state

    # If this is an event on the Aencoder pin
    if (channel == GPIO_Aencoder):
        encoderTicksA = encoderTicksA + 1                                           # Increase the tick count
        if (encoderTargetA > 0) and (encoderTicksA >= encoderTargetA):              # If we reached the target count, stop the motors and reset the count
            motorA_state = 0
    #print(encoderTicksA)


# Add an event to the encoder pin
GPIO.add_event_detect(GPIO_Aencoder, GPIO.RISING, callback = encoderCount, bouncetime=2)
encoderTicksA = 0
encoderTargetA = -1


# Wait function for a certain maximum time, or when the wheels have been stopped because the encoder count was reached
def waitForEncoder(maxTime):
    global motorA_state
    global encoderTargetA
    startTime = time.time()
    currentTime = startTime
    
    # Wait for the motors to have been stopped (by the encoder callback routine)
    # Also break from the routine if it took too long
    while (motorA_state != 0) and (currentTime-startTime < maxTime):
        currentTime = time.time()

    if (motorA_state != 0):                                 # We stopped because the maximum time was reached
        print("Motor A stopped because the time limit was reached")
        valid = False
    else:
        print("Motor A stopped - encoder target reached" )
        valid = True

    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, False)
    motorA_state = 0
    encoderTargetA = -1
            
    return valid


if __name__ == '__main__':
    try:
        
        while True:

            # Run the motor forward
            encoderTargetA = 39                             # Set the target for how much we want the motors to move
            encoderTicksA = 0                               # Reset the number of ticks thus far
            motorA_state = 1                                # The motor is running forward
            GPIO.output(GPIO_Ain1, True)
            GPIO.output(GPIO_Ain2, False)
            print ("Forward")

            valid = waitForEncoder(1000);                   # Wait for the encoderTarget to have been reached
                                                            # (or for at most 1000 seconds)
                                                            
            # Wait for 2 seconds
            time.sleep(2)

            # Run the motor backward
            encoderTargetA = 45                             # Set the target for how much we want the motors to move
            encoderTicksA = 0                               # Reset the number of ticks thus far
            motorA_state = -1                               # The motor is running backward
            GPIO.output(GPIO_Ain1, False)
            GPIO.output(GPIO_Ain2, True)
            print ("Backward")

            waitForEncoder(1000);                           # Wait for the encoderTarget to have been reached
                                                            # (or for at most 1000 seconds)

 
            
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        GPIO.cleanup()



