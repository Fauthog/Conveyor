from pynput.keyboard import Key, Listener
import threading
import RPi.GPIO as GPIO
import time



class conveyor_controller():
    def __init__(self):
        self.active:bool = False
        self.reverse:bool = False
        self.delay:float = 0.0
        self.driver_single_step:bool = False
        self.driver_shutdown:bool = False
        self.current_position:int = 0
        self.setpoint:int = 0
        self.closed_loop:bool = False

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
        self.length:float = 350.0
        self.steps:int = 1473
        self.x_min:int = 0
        self.x_max:int = self.steps
        self.mm_per_step:float = self.length / self.steps
        print("mm_per_step", self.mm_per_step)
        self.steps_per_rev:int = 400
        # Calculate delay based on RPM
        self.delay_low_speed:float = 60.0 / (self.steps_per_rev * 60)   
        self.delay_medium_speed:float = 60.0 / (self.steps_per_rev * 250) 
        self.delay_high_speed:float = 60.0 / (self.steps_per_rev * 600) 
        self.delay_single_step:float = 60.0 / (self.steps_per_rev * 10) 
        #  magic numbers end

        self.start_driver()

    def driver(self)->None:
        print("driver")
        while (True):
            time.sleep(1)
            if self.driver_shutdown:
                return
           
            # read variables
            active = self.active
            if active:
                reverse = self.reverse
                delay = self.delay
                single_step = self.driver_single_step
                closed_loop = self.closed_loop
                setpoint = self.setpoint
                # set direction pin
                GPIO.output(self.DIR_PIN, not reverse)
                # send pulse
                error = setpoint - self.current_position
                lower_limit_lock:bool = (self.current_position == self.x_min) and (reverse or (error<0))
                upper_limit_lock:bool = (self.current_position == self.x_max) and (not reverse or (error>0))

                if active and not lower_limit_lock and not upper_limit_lock: 
                    if not single_step and not closed_loop :
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        time.sleep(delay)
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                        time.sleep(delay)
                        print("pulse")
                    elif single_step: # send only one pulse to make singular step
                        # delay_single_step is fixed and will not be changed during runtime
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        time.sleep(self.delay_single_step)
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                        time.sleep(self.delay_single_step)
                        print("single_step")
                        self.driver_single_step=False
                        self.active=False
                        self.closed_loop = False
                    elif closed_loop and not single_step: 
                        print("closed_loop ", error)                   
                        if error != 0:
                            error_abs = abs(error)
                            if error_abs > 100:
                                delay=self.delay_medium_speed
                            if error_abs <= 100:
                                delay=self.delay_low_speed
                            if error >= 0:
                                reverse = False
                            if error < 0:
                                reverse = True
                            GPIO.output(self.STEP_PIN, GPIO.HIGH)
                            time.sleep(delay)
                            GPIO.output(self.STEP_PIN, GPIO.LOW)
                            time.sleep(delay)
                        else:
                            self.driver_single_step=False
                            self.active=False
                            self.closed_loop = False

                    if reverse:
                        self.current_position -= 1
                    else:
                        self.current_position +=1
                

    

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
        print("move", reverse, speed)
        self.reverse = reverse
        if speed == 1:
            self.delay=self.delay_low_speed
        if speed == 2:
            self.delay=self.delay_medium_speed
        if speed == 3:
            self.delay=self.delay_high_speed
        self.active=True       

    def move_raw(self, reverse:bool, delay:float)->None:
        print("move_raw")
        self.reverse=reverse
        if delay > 0:
            self.delay=delay
        self.active=True 
    
    def goto(self, position:float)->None:
        print("goto ", position)
        if position < self.x_min:
            position = self.x_min
        if position > self.x_max:
            position = self.x_max
        if (position >= self.x_min) and (position <= self.x_max):
            self.setpoint=position/self.mm_per_step
            self.closed_loop=True
            self.active=True
            
        
    def goto_min(self)->None:
        print("goto_min")
        self.setpoint=self.x_min
        self.closed_loop=True
        self.active=True

    def goto_max(self)->None:
        print("goto_max")
        self.setpoint=self.x_max
        self.closed_loop=True
        self.active=True
    
    def single_step(self, reverse:bool)->None:
        print("single_step")
        self.reverse = reverse
        self.driver_single_step = True
        self.active = True
    
    def stop(self)->None:
        print("stop")
        self.active = False

    def get_current_position(self)->float:
        print("get_current_position")
        return self.current_position * self.mm_per_step
     
    def zero(self)->None:
        print("zero")
        self.stop()
        self.current_position = 0
        print("current_position", self.current_position)
    
      
    # user input functions below
    def user_input(self)->None:
        self.lock:bool=False
        self.last_key:str=""

        with Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

    def on_press(self, key):
        # print(key)
        if key == Key.esc:
            self.stop()
            self.shutdown_driver()           
            return False
        
        if str(key) == "'z'":
            self.zero()
        
        if str(key) == "'+'":
            self.goto_max()

        if str(key) == "'-'":
            self.goto_min()

        if str(key) == "'s'":
            self.stop()
        
        if str(key) == "'g'":
            i=input()
            i.strip()
            try:
                g=int(i[-3:])            
                self.goto(g)
            except:
                print("NaN")

        if not self.lock:      
            # single step         
            if str(key) == "<103>":
                self.single_step(False)
                self.lock=True
                self.last_key=key

            if str(key) == "<105>":
                self.single_step(True)
                self.lock=True
                self.last_key=key
            
            #  move fast                
            if str(key) == "<100>":
                self.move(False, 2)
                self.lock=True
                self.last_key=key

            if str(key) == "<102>":
                self.move(True, 2)
                self.lock=True
                self.last_key=key

            #  move slow            
            if str(key) == "<97>":
                self.move(False, 1)
                self.lock=True
                self.last_key=key

            if str(key) == "<99>":
                self.move(True, 1)
                self.lock=True
                self.last_key=key
           
    def on_release(self, key):
        if key==self.last_key:
            self.lock=False
            
        if key == Key.esc:
            self.stop()
            self.shutdown_driver()  
            # Stop listener
            return False


def main()->None:   
    driver=conveyor_controller()
    driver.user_input()
    
if __name__ == "__main__":
    main()