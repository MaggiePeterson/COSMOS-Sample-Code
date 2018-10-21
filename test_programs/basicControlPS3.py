# Libraries
import RPi.GPIO as GPIO
import time
import pygame, sys
from time import sleep
import numpy as np

# setup the pygame window
pygame.init()
window = pygame.display.set_mode((200, 200), 0, 32)

# how many joysticks connected to computer?
joystick_count = pygame.joystick.get_count()
print("There is " + str(joystick_count) + " joystick/s")

if joystick_count == 0:
    # if no joysticks, quit program safely
    print ("Error, I did not find any joysticks")
    pygame.quit()
    sys.exit()
else:
    # initialise joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

axes = joystick.get_numaxes()
buttons = joystick.get_numbuttons()
hats = joystick.get_numhats()

def getAxis(number):
    # when nothing is moved on an axis, the VALUE IS NOT EXACTLY ZERO
    # so this is used not "if joystick value not zero"
    if joystick.get_axis(number) < -0.1 or joystick.get_axis(number) > 0.1:
      # value between 1.0 and -1.0
      print ("Axis value is %s" %(joystick.get_axis(number)))
      print ("Axis ID is %s" %(number))
 
def getButton(number):
    # returns 1 or 0 - pressed or not
    if joystick.get_button(number):
      # just prints id of button
      print ("Button ID is %s" %(number))

def getHat(number):
    if joystick.get_hat(number) != (0,0):
      # returns tuple with values either 1, 0 or -1
      print ("Hat value is %s, %s" %(joystick.get_hat(number)[0],joystick.get_hat(number)[1]))
      print ("Hat ID is %s" %(number))


# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
# set GPIO Pins
GPIO_Apwm = 6
GPIO_Ain1 = 13
GPIO_Ain2 = 19

GPIO_Bin1 = 16
GPIO_Bin2 = 20
GPIO_Bpwm = 21

GPIO_But1 = 12
GPIO_But2 = 5
GPIO_But3 = 23

# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)
GPIO.setup(GPIO_But1, GPIO.OUT)
GPIO.setup(GPIO_But2, GPIO.OUT)
GPIO.setup(GPIO_But3, GPIO.OUT)

# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)

# Set PWM parameters
pwm_frequency = 50

# Create the PWM instances
pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

# Set the duty cycle (between 0 and 100)
# The duty cycle determines the speed of the wheels
pwmA.start(100)
pwmB.start(100)

'''
# The PWM pins are set permanently to HIGH
# This sets the motor speed for both to the maximum value
# If we want to change the motor speed, we need to
# attach a PWM signal to these pins
GPIO.output(GPIO_Apwm, True)
GPIO.output(GPIO_Bpwm, True) 
'''

def set_duty_cycle(angle):
    pulse =  2*float(angle)/180.0 + 0.5
    duty = 0.1*pulse*pwm_frequency
    #duty = 2.5 + 0.12*float(angle) for frequency of 100
    return duty

# Create a PWM instance
pwm_servo1 = GPIO.PWM(GPIO_But1, pwm_frequency)
pwm_servo2 = GPIO.PWM(GPIO_But2, pwm_frequency)
pwm_servo3 = GPIO.PWM(GPIO_But3, pwm_frequency)

servo1Position = 0
servo2Position = 180


if __name__ == '__main__':
	try:
		
		while True:
			
			time.sleep(0.05)
			
			for event in pygame.event.get():
				# loop through events, if window shut down, quit program
				if event.type == pygame.QUIT:
					pygame.quit()
					sys.exit()

			j1 = joystick.get_axis(1)
			j3 = joystick.get_axis(3)
			print("axis 1: " + str(j1))
			print("axis 3: " + str(j3))
			
			pwm_servo1.start(set_duty_cycle(servo1Position))
			pwm_servo2.start(set_duty_cycle(servo2Position))
			#pwm_servo.start(set_duty_cycle(80))
			
			if joystick.get_button(15):
                                pwm_servo3.start(set_duty_cycle(120))
                                print("14 is pressed")
                                
                        if joystick.get_button(14):
                                pwm_servo3.start(set_duty_cycle(0))
                                print("15 is pressed")
                                
			
			if joystick.get_button(11):
                                servo1Position += 15
                                servo1Position = np.minimum(servo1Position, 180)
                                print("11 is pressed")
                                print("servo1Position: {}".format(servo1Position))
                                
                        if joystick.get_button(9):
                                servo1Position -= 15
                                servo1Position = np.maximum(servo1Position, 0)
                                print("9 is pressed")
                                print("servo1Position: {}".format(servo1Position))
                                
                        if joystick.get_button(10):
                                servo2Position -= 15
                                servo2Position = np.maximum(servo2Position, 0)
                                print("10 is pressed")
                                print("servo2Position: {}".format(servo2Position))
                                
                        if joystick.get_button(8):
                                servo2Position += 15
                                servo2Position = np.minimum(servo2Position, 180)
                                print("8 is pressed")
                                print("servo2Position: {}".format(servo2Position))
			 
			if j1 > 0.1:
				GPIO.output(GPIO_Ain1, True)
				GPIO.output(GPIO_Ain2, False)
			else:
				if j1 < -0.1:
					GPIO.output(GPIO_Ain1, False)
					GPIO.output(GPIO_Ain2, True)
					
				else:
					GPIO.output(GPIO_Ain1, False)
					GPIO.output(GPIO_Ain2, False)
					
			pwmA.ChangeDutyCycle(int(np.abs(j1)*100))

			if j3 > 0.1:
				GPIO.output(GPIO_Bin1, True)
				GPIO.output(GPIO_Bin2, False)
			else:
				if j3 < -0.1:
					GPIO.output(GPIO_Bin1, False)
					GPIO.output(GPIO_Bin2, True)
					
				else:
					GPIO.output(GPIO_Bin1, False)
					GPIO.output(GPIO_Bin2, False)

			pwmB.ChangeDutyCycle(int(np.abs(j3)*100))
			            
        # Reset by pressing CTRL + C
	except KeyboardInterrupt:
		print("Program stopped by User")
		GPIO.cleanup()
		pygame.quit()
		sys.exit()
