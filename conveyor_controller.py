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
        self.closed_loop:bool = False
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
        self.max_acceleration_per_step = 250 # max 250 rpm/step 
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
                closed_loop = self.closed_loop

                if closed_loop:
                    position_setpoint = self.position_setpoint # position setpoint in pulse space
                    position = self.current_position # current position in pulse space
                    speed_setpoint = self.speed_setpoint # speed setpoint in rpm
                    # simple controller
                    position_error = position_setpoint - position                   
                    error_abs = abs(position_error)
                    
                    if 100 <= error_abs <=200:
                        speed_setpoint = self.medium_speed
                    if error_abs <= 100:
                        speed_setpoint = self.low_speed
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


                    lower_limit_lock:bool = (self.current_position == self.x_min) and (limited_speed < 0)
                    upper_limit_lock:bool = (self.current_position == self.x_max) and (limited_speed > 0)
                    if not lower_limit_lock or upper_limit_lock:
                        if limited_speed < 0:
                            GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                            delay:float = 60.0 / (self.steps_per_rev * abs(limited_speed))
                            GPIO.output(self.STEP_PIN, GPIO.HIGH)
                            time.sleep(delay)
                            self.current_position -= 1
                            GPIO.output(self.STEP_PIN, GPIO.LOW)
                            time.sleep(delay)

                        elif limited_speed > 0:
                            GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                            delay:float = 60.0 / (self.steps_per_rev * limited_speed)
                            GPIO.output(self.STEP_PIN, GPIO.HIGH)
                            time.sleep(delay)
                            self.current_position +=1
                            GPIO.output(self.STEP_PIN, GPIO.LOW)
                            time.sleep(delay)
                        
                        current_speed = limited_speed

                if not closed_loop:
                        reverse = self.reverse
                        speed_setpoint = self.speed_setpoint
                        # we are continuously moving the belt in a given direction at a given speed 
                        delay:float = 60.0 / (self.steps_per_rev * speed_setpoint)
                        GPIO.output(self.DIR_PIN, not reverse)
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        time.sleep(delay)
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                        time.sleep(delay)
                        print("pulse")
                        if reverse:
                            self.current_position -= 1                        
                        else:
                            self.current_position +=1
                    
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

    def move(self, reverse:bool, speed:int)->None:
        self.stop()
        print("move", reverse, speed)
        self.closed_loop = False
        self.reverse = reverse
        if speed == 1:
            self.speed_setpoint = self.low_speed
        if speed == 2:
            self.speed_setpoint = self.medium_speed
        if speed == 3:
            self.speed_setpoint = self.high_speed
        self.active=True       

    def move_raw(self, reverse:bool, speed:float)->None:
        self.stop()
        print("move_raw")
        self.closed_loop = False
        self.reverse=reverse
        if speed > 0 and speed <= self.high_speed:
            self.speed_setpoint = speed
        self.active=True 

    def goto(self, position:float,  speed:int)->None:
        # posiiont as float value between 0 and self.length
        print("goto ", position)
        # clamp position to within the limits
        if position < self.x_min:
            position = self.x_min
        if position > self.x_max:
            position = self.x_max
        if (position >= self.x_min) and (position <= self.x_max):
            if speed == 1:
                self.speed_setpoint = self.low_speed
            if speed == 2:
                self.speed_setpoint = self.medium_speed
            if speed == 3:
                self.speed_setpoint = self.high_speed
            self.position_setpoint=position/self.mm_per_step
            self.closed_loop=True
            self.active=True

    def goto_plus_x(self, x:float,  speed:int)->None:
        # posiiont as float value between 0 and self.length
        print("goto plus x ", x)
        # clamp position to within the limits
        position = self.get_current_position() + x
        if position < self.x_min:
            position = self.x_min
        if position > self.x_max:
            position = self.x_max
        if (position >= self.x_min) and (position <= self.x_max):
            if speed == 1:
                self.speed_setpoint = self.low_speed
            if speed == 2:
                self.speed_setpoint = self.medium_speed
            if speed == 3:
                self.speed_setpoint = self.high_speed
            self.position_setpoint=position/self.mm_per_step
            self.closed_loop=True
            self.active=True
    
    def goto_with_speed(self, position:float, speed:int)->None:
        # posiiont as float value between 0 and self.length, speed in int rpm 
        print("goto ", position)
        # clamp position to within the limits
        if position < self.x_min:
            position = self.x_min
        if position > self.x_max:
            position = self.x_max
        if (position >= self.x_min) and (position <= self.x_max):
            self.speed_setpoint = 60.0 / (self.steps_per_rev * speed)
            self.position_setpoint=position/self.mm_per_step
            self.closed_loop=True
            self.active=True
            
        
    def goto_min(self)->None:
        print("goto_min")
        self.goto(self.x_min, 2)
       

    def goto_max(self)->None:
        print("goto_max")
        self.goto(self.x_max, 2)

    
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
    
      
    


def main()->None:   
    driver=conveyor_controller()
    
    
if __name__ == "__main__":
    main()