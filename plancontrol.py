import RPi.GPIO as GPIO
import time
import numpy as np
import threading
import KeypressModule as kp



# Define GPIO pins
STEP_PIN = 18
DIR_PIN = 27
LIMIT_SWITCH_PIN_LEFT = 6  # Limit switch at the left side of the conveyor
LIMIT_SWITCH_PIN_RIGHT = 13  # Limit switch at the right side of the conveyor

# Define stepper motor parameters
STEPS_PER_REV = 400  # Steps per revolution

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(LIMIT_SWITCH_PIN_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up resistor
GPIO.setup(LIMIT_SWITCH_PIN_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up resistor

# Global variables for speed and direction
speed = None
direction = None
motor_running = False
kp.init()

# continuous movement
Final_distance = 1473  # Length of belt in mm
# SPEED = 0.3  # Constant speed in m/s

def move_fwd_rapid():
	
    # Calculate delay based on RPM
	delay = 60.0 / (STEPS_PER_REV * 250)           
    
    # Set direction
	GPIO.output(DIR_PIN, GPIO.HIGH)
        # Check if either limit switch is triggered
	if GPIO.input(LIMIT_SWITCH_PIN_RIGHT) == GPIO.LOW :
		stop_motor()
		print("Limit switch triggered. Stopping motor.")
		return
	GPIO.output(STEP_PIN, GPIO.HIGH)
	time.sleep(delay)
	GPIO.output(STEP_PIN, GPIO.LOW)
	time.sleep(delay)

	print("Move rapid forward")
		
	
def move_bkw_rapid():
    # Calculate delay based on RPM
	delay = 60.0 / (STEPS_PER_REV * 250)          
    
    # Set direction
	GPIO.output(DIR_PIN, GPIO.LOW)
        # Check if either limit switch is triggered
	if GPIO.input(LIMIT_SWITCH_PIN_LEFT) == GPIO.LOW :
		stop_motor()
		print("Limit switch triggered. Stopping motor.")
		return
	GPIO.output(STEP_PIN, GPIO.HIGH)
	time.sleep(delay)
	GPIO.output(STEP_PIN, GPIO.LOW)
	time.sleep(delay)
	print("Move rapid backward")

	

def move_fwd_adjust():
    # Calculate delay based on RPM
	delay = 60.0 / (STEPS_PER_REV *60)

    # Set direction
	GPIO.output(DIR_PIN, GPIO.HIGH)
        # Check if either limit switch is triggered
	if GPIO.input(LIMIT_SWITCH_PIN_RIGHT) == GPIO.LOW :
		stop_motor()
		print("Limit switch triggered. Stopping motor.")
		return
	GPIO.output(STEP_PIN, GPIO.HIGH)
	time.sleep(delay)
	GPIO.output(STEP_PIN, GPIO.LOW)
	time.sleep(delay)
	print("Move adjust forward")
	
	
def move_bkw_adjust():	
    # Calculate delay based on RPM
	delay = 60.0 / (STEPS_PER_REV * 60)
    
    # Set direction
	GPIO.output(DIR_PIN, GPIO.LOW)
	        # Check if either limit switch is triggered
	if GPIO.input(LIMIT_SWITCH_PIN_LEFT) == GPIO.LOW :
		stop_motor()
		print("Limit switch triggered. Stopping motor.")
		return
	GPIO.output(STEP_PIN, GPIO.HIGH)
	time.sleep(delay)
	GPIO.output(STEP_PIN, GPIO.LOW)
	time.sleep(delay)
	print("Move rapid backward")

def set_lowend():
    global motor_running
    motor_running = True
    # Calculate delay based on RPM
    delay = 60.0 / (STEPS_PER_REV * 60)
    speed = 350 / (400 * 1473) * 1 / delay
    
    # Set direction
    GPIO.output(DIR_PIN, GPIO.LOW)
    
    # Move the motor continuously until the desired distance is reached
    distance_travelled = 0
    while motor_running:
		# Check if the 's' key is pressed to stop the motor
        if kp.getKey('s'):
            stop_motor()
            break
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)
        print("Motor running" , speed , " mm/s")
        
        # Update distance travelled
        distance_travelled += 1
        print("distance" , distance_travelled)
        # Check if the desired distance is reached
        if distance_travelled >= Final_distance:
            motor_running = False
            break  # Exit the loop if desired distance is reached
    
    print("Motor stopped at the high end")

	
def set_highend():
    global motor_running
    motor_running = True
    # Calculate delay based on RPM
    delay = 60.0 / (STEPS_PER_REV * 60)
    speed = 350 / (400 * 1473) * 1 / delay
    
    # Set direction
    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    # Move the motor continuously until the desired distance is reached
    distance_travelled = 0
    while motor_running:
		 # Check if the 's' key is pressed to stop the motor
        if kp.getKey('s'):
            stop_motor()
            break
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)
        print("distance" , distance_travelled)
        
        # Update distance travelled
        distance_travelled += 1
        print("distance" , distance_travelled)
        
        # Check if the desired distance is reached
        if distance_travelled >= Final_distance:
            motor_running = False
            break  # Exit the loop if desired distance is reached
    
    print("Motor stopped at the high end")
	

def stop_motor():
    global motor_running
    if motor_running:
        motor_running = False
        print("Motor stopped.")
    else:
        print("Motor is not running.")
       

# Main program loop
try:
	pulse = 0
	while True:
			if kp.getKey('LEFT'):
				move_fwd_adjust()
				pulse -= 1
					
			elif kp.getKey('RIGHT'):
				move_bkw_adjust()
				pulse += 1
			elif kp.getKey('PAGEUP'):
				move_fwd_rapid()
			elif kp.getKey('PAGEDOWN'):
				move_bkw_rapid()
			elif kp.getKey('l'):
				set_lowend()
			elif kp.getKey('h'):
				set_highend()
			
			elif kp.getKey('s'):
				stop_motor()
			# else:
				# # stop_motor()
			# print("pulse:", pulse)
except KeyboardInterrupt:
    stop_motor()
finally:
    GPIO.cleanup()
