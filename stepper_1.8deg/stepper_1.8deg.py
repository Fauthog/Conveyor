import RPi.GPIO as GPIO
import time

# Define GPIO pins
STEP_PIN = 18
DIR_PIN = 27

# Define stepper motor parameters
STEPS_PER_REV = 400  # 400 Steps per revolution via microstep as per datasheet (https://www.yakotec.com/index.php/proview-8-6.html)
ROUND_TO_MM = 95.2 # Based on the current diameter of the pulley, measured from the actual linear displacement
MAX_RPM = 600 # Tested to generate correct waveform on PI5 (smallest step delay=250uS)

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# Hi-res sleep taken from https://stackoverflow.com/questions/1133857/how-accurate-is-pythons-time-sleep
def high_precision_sleep(duration):
	start_time = time.perf_counter()
	while True:
		elapsed_time = time.perf_counter() - start_time
		remaining_time = duration - elapsed_time
		if remaining_time <= 0:
			break
		if remaining_time > 0.02:  # Sleep for 5ms if remaining time is greater
			time.sleep(max(remaining_time/2, 0.0001))  # Sleep for the remaining time or minimum sleep interval
		else:
			pass
# Calculate step delay based on the given rpm
def rpm_to_step_delay(rpm):
	# Given then RPM, divide it by 60 to convert to RPS
	# then multiply it by STEPS_PER_REV to get SPS.
	# The inverse is the time delay in second
	#sps = (rpm/60)*STEPS_PER_REV
	#print(rpm, "RPM = ", sps, "Steps/sec. Step delay =", 1/sps, "sec")
	#return 1/sps
	return 60/(rpm*STEPS_PER_REV)
	
# Move the motor using given direction, steps, and rpm
def move_steps(direction, steps, rpm):
	# Set direction
	if direction == "forward":
		GPIO.output(DIR_PIN, GPIO.LOW)
	elif direction == "backward":
		GPIO.output(DIR_PIN, GPIO.HIGH)
	else:
		print("Invalid direction. The motor will not move")
		return
		
	# Calculate step delay
	step_delay = rpm_to_step_delay(rpm)

	# Move the motor
	for i in range(steps):
		GPIO.output(STEP_PIN, GPIO.HIGH)
		high_precision_sleep(step_delay/2)
		GPIO.output(STEP_PIN, GPIO.LOW)
		high_precision_sleep(step_delay/2)

# Move the motor using given direction, distance, and rpm
def move_mm(direction, distance, rpm):
	# Calculate required steps
	steps = int(distance*400/ROUND_TO_MM)	# Round to int can cause ca. 0.12mm error
	move_steps(direction, steps, rpm)
	
# Start here
time_start = time.perf_counter()
for i in range(10):
	move_steps("forward", 400, 60)
	move_mm("backward", ROUND_TO_MM, 60)
	print("Iteration:", i)
	
# Calculate time difference
print("Elapse time =", time.perf_counter() - time_start, "sec")
