# This program demonstrates the use of the PCA9685 PWM driver
# This is useful to effectively control multiple servos
# In this example, there is a servo on channel 0 and channel 1

# Libraries
import time
import Adafruit_PCA9685

# GPIO library not necessary if we only use the PWM driver
#import RPi.GPIO as GPIO

 
# GPIO Mode (BOARD / BCM)
#GPIO.setmode(GPIO.BOARD)


# Initialise the PCA9685 PWM driver using the default address (0x40)
# and set the pwm parameters
pwm = Adafruit_PCA9685.PCA9685()
pwm_frequency = 50
pwm.set_pwm_freq(pwm_frequency)
servo_min = (145 * pwm_frequency) // 50
servo_max = (580 * pwm_frequency) // 50


# Function to calculate the servo pulse width (number between 0 and 4095)
def servoSetting(angle):
    return ((servo_max - servo_min) * angle//180 + servo_min)




 
if __name__ == '__main__':
    try:
        
        while True:
            angle = 0
            channel = 0
            pwm.set_pwm(channel, 0, servoSetting(angle))
            print ('angle: {0} \t channel: {1}'.format(angle,channel))
            time.sleep(1)

            angle = 90
            channel = 0
            pwm.set_pwm(channel, 0, servoSetting(angle))
            print ('angle: {0} \t channel: {1}'.format(angle,channel))
            time.sleep(1)

            angle = 0
            channel = 1
            pwm.set_pwm(channel, 0, servoSetting(angle))
            print ('angle: {0} \t channel: {1}'.format(angle,channel))
            time.sleep(1)

            angle = 90
            channel = 1
            pwm.set_pwm(channel, 0, servoSetting(angle))
            print ('angle: {0} \t channel: {1}'.format(angle,channel))
            time.sleep(1)

            angle = 0
            channel = 2
            pwm.set_pwm(channel, 0, servoSetting(angle))
            print ('angle: {0} \t channel: {1}'.format(angle,channel))
            time.sleep(1)

            angle = 90
            channel = 2
            pwm.set_pwm(channel, 0, servoSetting(angle))
            print ('angle: {0} \t channel: {1}'.format(angle,channel))
            time.sleep(1)

 
            
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        #GPIO.cleanup()