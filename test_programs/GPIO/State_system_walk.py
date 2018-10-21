# This program illustrates how to use an FSM to time the servo
# It also has an LED that starts blinking when the button is pressed, showing
# how you can have more than one timed element

# Libraries
import RPi.GPIO as GPIO
import time
import Adafruit_PCA9685
 
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)
 
# set GPIO Pins
GPIO_Servo = 7
BtnPin = 15                                     


# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Servo, GPIO.OUT)
GPIO.setup(BtnPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)     # Set BtnPin's mode is input, and pull up to high level (3.3V)



# Set PWM parameters
pwm = Adafruit_PCA9685.PCA9685()
pwm_frequency = 50
pwm.set_pwm_freq(pwm_frequency)
servo_min = (145 * pwm_frequency) // 50
servo_max = (580 * pwm_frequency) // 50

def servoSetting(angle):
    return ((servo_max - servo_min) * angle//180 + servo_min)



# ------------------------------------------------------------
# Keep track of the state
servoState = 0
servoNextState = 0

# Servo move interval in seconds
servoInterval = 1
servoLastTime = 0

if __name__ == '__main__':
    try:
        
        while True:

            time.sleep(0.01)
            currentTime = time.time()
            
            servoState = servoNextState
            
            # Check the state transition for the servo
            if (servoState == 0):
                
                if (currentTime - servoLastTime > servoInterval):

                    print("delay: " + str(currentTime - servoLastTime))

                    angle = 0
                    channel = 0
                    pwm.set_pwm(channel, 0, servoSetting(angle))
                    print ('angle: {0} \t channel: {1}'.format(angle,channel))
                    
                    angle = 0
                    channel = 1
                    pwm.set_pwm(channel, 0, servoSetting(angle))
                    print ('angle: {0} \t channel: {1}'.format(angle,channel))
                    
                    angle = 0
                    channel = 2
                    pwm.set_pwm(channel, 0, servoSetting(angle))
                    print ('angle: {0} \t channel: {1}'.format(angle,channel))
                    
                    angle = 0
                    channel = 3
                    pwm.set_pwm(channel, 0, servoSetting(angle))
                    print ('angle: {0} \t channel: {1}'.format(angle,channel))
                    
                    servoNextState = 1
                    servoLastTime = currentTime

                else:
                    servoNextState = 0

            elif (servoState == 1):
                
                if (currentTime - servoLastTime > servoInterval):
                
                    angle = 45
                    channel = 0
                    pwm.set_pwm(channel, 0, servoSetting(angle))
                    print ('angle: {0} \t channel: {1}'.format(angle,channel))
                    
                    angle = 45
                    channel = 1
                    pwm.set_pwm(channel, 0, servoSetting(angle))
                    print ('angle: {0} \t channel: {1}'.format(angle,channel))
                    
                    angle = 45
                    channel = 2
                    pwm.set_pwm(channel, 0, servoSetting(angle))
                    print ('angle: {0} \t channel: {1}'.format(angle,channel))
                    
                    angle = 45
                    channel = 3
                    pwm.set_pwm(channel, 0, servoSetting(angle))
                    print ('angle: {0} \t channel: {1}'.format(angle,channel))
                    
                    servoNextState = 0
                    servoLastTime = currentTime
                    
                else:
                    servoNextState = 1
                    
                    
            else:
                print("Error: unrecognized state")
                servoNextState = servoState




                    

            
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        GPIO.cleanup()