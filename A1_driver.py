from pynput.keyboard import Key, Listener
import threading
import RPi.GPIO as GPIO
import time



class conveyor_controller():
	def __init__(self):
		self.active:bool = False
		self.reverse:bool = False
		self.driver_shutdown:bool = False
		self.current_position:int = 0
		self.position_setpoint:int = 0        
		self.speed_setpoint:int = 0        
		# setup GPIO
		# Define GPIO pins
		self.STEP_PIN = 18
		self.DIR_PIN = 27

		# LIMIT_SWITCH_PIN_LEFT = 6  # Limit switch at the left side of the conveyor
		# LIMIT_SWITCH_PIN_RIGHT = 13  # Limit switch at the right side of the conveyor

		# # Setup GPIO
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.STEP_PIN, GPIO.OUT)
		GPIO.setup(self.DIR_PIN, GPIO.OUT)
		# # GPIO.setup(LIMIT_SWITCH_PIN_LEFT, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up resistor
		# # GPIO.setup(LIMIT_SWITCH_PIN_RIGHT, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up resistor
		
		# magic numbers start
		self.steps_per_rev:int = 400
		self.length:float = 350.0
		self.steps:int = 1473
		self.x_min:int = 0
		self.x_max:int = self.steps
		self.mm_per_step:float = self.length / self.steps
		
		# pre-defined limits. These can be changed
		self.low_speed:int = 60
		self.medium_speed:int = 250
		self.high_speed:int = 600
		self.max_acceleration_per_step = 1 # max 250 rpm/step 
		#  magic numbers end

		self.start_driver()


	def driver(self)->None:
		print("driver")
		current_speed:int = 0
		position_error:float = 0.0
		speed_setpoint:int = 0
		while (True):
			if self.driver_shutdown:
				return

			active = self.active
			if active:
				position_setpoint = self.position_setpoint # position setpoint in pulse space
				position = self.current_position # current position in pulse space
				speed_setpoint = self.speed_setpoint # speed setpoint in rpm
				
				# simple controller
				position_error = position_setpoint - position                   
				error_abs = abs(position_error)
				
				if 100 <= error_abs <=200:
					speed_setpoint = min(self.medium_speed, self.speed_setpoint)
				if error_abs <= 100:
					speed_setpoint = min(self.low_speed, self.speed_setpoint)
				if error_abs < 1:
					self.active = False
				if position_error > 0:
						reverse = False
				if position_error < 0:
						reverse = True
						speed_setpoint = -speed_setpoint # if we need to move in reverse, we have to use a negative speed_setpoint
				# limit acceleration
				speed_error = speed_setpoint - current_speed
				if speed_error < -self.max_acceleration_per_step:
					limited_speed = current_speed - self.max_acceleration_per_step
				elif speed_error > self.max_acceleration_per_step:
					limited_speed = current_speed + self.max_acceleration_per_step
				else:
					limited_speed = current_speed + speed_error

				print("driver", position_error, limited_speed)
				lower_limit_lock:bool = (self.current_position == self.x_min) and (limited_speed < 0)
				upper_limit_lock:bool = (self.current_position == self.x_max) and (limited_speed > 0)
				if not lower_limit_lock or upper_limit_lock:
					if limited_speed < 0:
						GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
						delay:float = 60.0 / (self.steps_per_rev * abs(limited_speed))
						GPIO.output(self.STEP_PIN, GPIO.HIGH)
						self.high_precision_sleep(delay)
						self.current_position -= 1
						GPIO.output(self.STEP_PIN, GPIO.LOW)
						self.high_precision_sleep(delay)

					elif limited_speed > 0:
						GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
						delay:float = 0.5 * 60.0 / (self.steps_per_rev * limited_speed)
						GPIO.output(self.STEP_PIN, GPIO.HIGH)
						self.high_precision_sleep(delay)
						self.current_position +=1
						GPIO.output(self.STEP_PIN, GPIO.LOW)
						self.high_precision_sleep(delay)
					
					current_speed = limited_speed                

			else:
				current_speed = 0
				position_error = 0.0



	def start_driver(self)->None:
		print("start_driver")
		self.driver_shutdown=False
		self.driver_thread = threading.Thread(target=self.driver)
		self.driver_thread.start()

	def shutdown_driver(self)->None:
		print("shutdown_driver")
		self.driver_shutdown=True
		self.driver_thread.join()
		return

	def goto(self, position:float, speed:int)->None:
		# position as float value between 0 and self.length
		print("goto ", position)
		# clamp position to within the limits
		if position < self.x_min:
			position = self.x_min
		if position > self.x_max:
			position = self.x_max
		if self.x_min<= position <= self.x_max:
			self.speed_setpoint = speed
			self.position_setpoint=position/self.mm_per_step 
			self.active=True

	def goto_plus_x(self, x:float, speed:int)->None:
		# position as float value between 0 and self.length
		print("goto plus x ", x)
		# clamp position to within the limits
		position = self.get_current_position() + x
		if position < self.x_min:
			position = self.x_min
		if position > self.x_max:
			position = self.x_max
		if self.x_min<= position <= self.x_max:
			self.speed_setpoint = speed
			self.position_setpoint=position/self.mm_per_step
			self.active=True  


	def goto_min(self)->None:
		print("goto_min")
		self.goto(self.x_min, 2)
	   

	def goto_max(self)->None:
		print("goto_max")
		self.goto(self.x_max, 2)
	
	
	def move(self, position:float, speed:int)->None:
		print("move ")
		self.speed_setpoint = speed
		self.position_setpoint=position/self.mm_per_step 
		self.active=True

	def stop(self)->None:
		print("stop")
		self.active = False
		self.position_setpoint=self.get_current_position
		time.sleep(0.01)

	def get_current_position(self)->float:
		print("get_current_position")
		return self.current_position * self.mm_per_step
	 
	def zero(self)->None:
		print("zero")
		self.stop()
		self.current_position = 0
		print("current_position", self.current_position)


	def high_precision_sleep(self, duration:float)->None:
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



def main()->None:   
	driver=conveyor_controller()
	# Start here
	driver.zero()
	time_start = time.perf_counter()
	driver.move(1000000, 400)
	
	
	
	#for i in range(3):
	#	driver.goto(300, 400)
	#	time.sleep(0.2)
	#	driver.goto(300, 200)
	#	time.sleep(1)
	#	driver.goto(0, 400)
	#	print("Iteration:", i)
	#	time.sleep(1)
	
	# Calculate time difference
	print("Elapse time =", time.perf_counter() - time_start, "sec")
	# driver.shutdown_driver()

if __name__ == "__main__":
    main()
