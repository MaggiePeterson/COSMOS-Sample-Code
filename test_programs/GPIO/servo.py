# Libraries
import RPi.GPIO as GPIO
import time
 
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
# set GPIO Pins
GPIO_Servo = 12

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Servo, GPIO.OUT)



# Set PWM parameters
pwm_frequency = 50

def set_duty_cycle(angle):
    pulse =  2*float(angle)/180.0 + 0.5
    duty = 0.1*pulse*pwm_frequency
    #duty = 2.5 + 0.12*float(angle) for frequency of 100
    return duty

    
# Create a PWM instance
pwm_servo = GPIO.PWM(GPIO_Servo, pwm_frequency)

# Set the duty cycle (between 0 and 100)
angle = 90
pwm_servo.start(set_duty_cycle(angle))

 
if __name__ == '__main__':
    try:
        
        while True:
            angle = 0
            pwm_servo.start(set_duty_cycle(angle))
            print ("0")
            time.sleep(1)
            
            angle = 45
            pwm_servo.start(set_duty_cycle(angle))
            print ("45")
            time.sleep(1)
            
            angle = 90
            pwm_servo.start(set_duty_cycle(angle))
            print ("90")
            time.sleep(1)

            angle = 135
            pwm_servo.start(set_duty_cycle(angle))
            print ("135")
            time.sleep(1)

            angle = 180
            pwm_servo.start(set_duty_cycle(angle))
            print ("180")
            time.sleep(1)
            
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        GPIO.cleanup()
     