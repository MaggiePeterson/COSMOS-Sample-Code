# Libraries
import RPi.GPIO as GPIO
import time

 
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
# set GPIO Pins
GPIO_Apwm = 6
GPIO_Ain1 = 13
GPIO_Ain2 = 19

GPIO_Bin1 = 16
GPIO_Bin2 = 20
GPIO_Bpwm = 21

# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)

# The PWM pins are set permanently to HIGH
# This sets the motor speed for both to the maximum value
# If we want to change the motor speed, we need to
# attach a PWM signal to these pins
GPIO.output(GPIO_Apwm, True)
GPIO.output(GPIO_Bpwm, True) 


if __name__ == '__main__':
    try:
        
        while True:

            GPIO.output(GPIO_Ain1, True)
            GPIO.output(GPIO_Ain2, False)
            GPIO.output(GPIO_Bin1, True)
            GPIO.output(GPIO_Bin2, False)
            print ("Forward")
            time.sleep(1)
            
            GPIO.output(GPIO_Ain1, False)
            GPIO.output(GPIO_Ain2, True)
            GPIO.output(GPIO_Bin1, False)
            GPIO.output(GPIO_Bin2, True)
            print ("Backward")
            time.sleep(1)

            GPIO.output(GPIO_Ain1, False)
            GPIO.output(GPIO_Ain2, False)
            GPIO.output(GPIO_Bin1, False)
            GPIO.output(GPIO_Bin2, False)
            print ("Stop")
            time.sleep(1)
            
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        GPIO.cleanup()
