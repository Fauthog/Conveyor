from threading import Thread
import RPi.GPIO as GPIO
import time
from multiprocessing import Process, Queue
import cv2
from PIL import Image
from picamera2 import Picamera2, Preview
import numpy as np



class governor():
    def __init__(self):            
        self.process_list = []
        self.QueueShutdown = Queue()
        self.QueueStop = Queue()
        self.QueueSetPoint = Queue()
        self.QueueCurrentPosition = Queue()
        self.QueueCurrentFrame = Queue()
        self.QueueCorrectionFromCV = Queue()
        self.current_position_in_mm:float = 0.0
        self.convertedImage = None
        self.ResetState:bool = False
        self.state:str = "reset"
        self.speedFactor:float = 1.0
        self.continuesMode:bool = False
       

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
        self.max_acceleration_per_step = 25 # max 250 rpm/step 
        #  magic numbers end

        #self.startup()


    def driver(self, QueueShutdown, QueueStop, QueueSetpoint, QueueCurrentPosition)->None:
        print("driver")
        shutdown:bool = False
        stop:bool = True
        current_position:int = 0
        # setup GPIO
        # Define GPIO pins
        self.STEP_PIN = 18
        self.DIR_PIN = 27
        LL_PIN = 6

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        # GPIO.setup(LL_PIN, GPIO.IN)
        GPIO.setup(LL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up resistor

        current_speed:int = 0
        position_error:float = 0.0
        speed_setpoint:int = 0
        position_setpoint:int = 0

        while (True):
            start_time = time.perf_counter()
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown driver")              
                return

            if not QueueStop.empty():
                stop = QueueStop.get()
            if not QueueSetpoint.empty():
                position_setpoint, speed_setpoint = QueueSetpoint.get()
                #print("driver", position_setpoint, speed_setpoint, current_position)

            # position_error = position_setpoint - current_position     
          
            position_error = position_setpoint - current_position 
            if (GPIO.input(LL_PIN) ==0) and (position_error<0):
                stop = True
                current_position = 0

            if not stop:              
                
                # simple controller
                                  
                error_abs = abs(position_error)
                
                if 50 <= error_abs <=100:
                    speed_setpoint = min(self.medium_speed, speed_setpoint)
                if error_abs <= 10:
                    speed_setpoint = min(self.low_speed, speed_setpoint)
                if error_abs < 1:
                    speed_setpoint = 0
                    current_speed = 0    
                if position_error < 0:
                        speed_setpoint = -speed_setpoint # if we need to move in reverse, we have to use a negative speed_setpoint
                # limit acceleration

                speed_error = speed_setpoint - current_speed
                if speed_error < -self.max_acceleration_per_step:
                    limited_speed = current_speed - self.max_acceleration_per_step
                elif speed_error > self.max_acceleration_per_step:
                    limited_speed = current_speed + self.max_acceleration_per_step
                else:
                    limited_speed = current_speed + speed_error

                # print("driver", position_error, current_position, limited_speed, speed_error)
                lower_limit_lock = False
                upper_limit_lock = False
                # lower_limit_lock:bool = (current_position == self.x_min) and (limited_speed < 0)
                # upper_limit_lock:bool = (current_position == self.x_max) and (limited_speed > 0)
                # if lower_limit_lock:
                #     print("lower", current_speed, limited_speed)
                #     limited_speed = 0
                #     speed_setpoint = 0
                #     current_speed = limited_speed
                if not lower_limit_lock or upper_limit_lock:
                    if limited_speed < 0:
                        # GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                        # delay:float = 0.5 * 60.0 / (self.steps_per_rev * abs(limited_speed))
                        # GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        # #self.high_precision_sleep(delay)
                        # time.sleep(delay)
                        # current_position -= 1
                        # GPIO.output(self.STEP_PIN, GPIO.LOW)
                        # #self.high_precision_sleep(delay)
                        # time.sleep(delay)

                        GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                        delay:float = 0.5 * 60.0 / (self.steps_per_rev * abs(limited_speed))
                        call_time_delta = time.perf_counter() - start_time
                        delta_delay = delay - call_time_delta
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                        #self.high_precision_sleep(delay)
                        if 0 <= delta_delay <0.03:
                            time.sleep(delta_delay)
                        else:
                            time.sleep(delay)
                        # time.sleep(delay)
                        current_position -= 1
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        #self.high_precision_sleep(delay)
                        time.sleep(delay)
                        GPIO.output(self.STEP_PIN, GPIO.LOW)

                    elif limited_speed > 0:
                        # GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                        # delay:float = 0.5 * 60.0 / (self.steps_per_rev * limited_speed)
                        # GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        # #self.high_precision_sleep(delay)
                        # time.sleep(delay)
                        # current_position +=1
                        # GPIO.output(self.STEP_PIN, GPIO.LOW)
                        # #self.high_precision_sleep(delay)
                        # time.sleep(delay)

                        GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                        delay:float = 0.5 * 60.0 / (self.steps_per_rev * abs(limited_speed))
                        call_time_delta = time.perf_counter() - start_time
                        delta_delay = delay - call_time_delta
                        # print("d", delta_delay)
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                        #self.high_precision_sleep(delay)
                        if 0 <= delta_delay <0.03:
                            time.sleep(delta_delay)
                        else:
                            time.sleep(delay)
                        current_position += 1
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        #self.high_precision_sleep(delay)
                        time.sleep(delay)
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                    
                    current_speed = limited_speed
                                   

            else:
                # ramp down to stop
                if current_speed > 1:  
                    limited_speed = max(current_speed-60, 1) 
                    GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * limited_speed)
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    #self.high_precision_sleep(delay)
                    time.sleep(delay)
                    current_position +=1
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    #self.high_precision_sleep(delay)
                    time.sleep(delay)
                    current_speed = limited_speed

                elif current_speed < -1:
                    limited_speed = min(current_speed+60, -1)
                    GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * abs(limited_speed))
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    #self.high_precision_sleep(delay)
                    time.sleep(delay)
                    current_position -= 1
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    #self.high_precision_sleep(delay)
                    time.sleep(delay)
                    current_speed = limited_speed
                else:
                    # if we are stopped, we wait for stop:bool to be false
                    time.sleep(0.05)
            QueueCurrentPosition.put(current_position) 
				
    def current_position_converter(self,QueueShutdown, QueueCurrentPosition)->None:
        print("convert position")
        shutdown:bool = False
        while True:
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown convert position")              
                return
            if not QueueCurrentPosition.empty():
                position = QueueCurrentPosition.get()
                #while(not QueueCurrentPosition.empty()):                   
                #    try:
                #        position = QueueCurrentPosition.get()
                #    except:
                #        ...
                self.current_position_in_mm = position * self.mm_per_step
            else:
                time.sleep(0.005)

    def get_current_position(self)->float:        
        return self.current_position_in_mm 
    
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

    def start_controller(self)->None:
        self.ResetState = False
        self.QueueStop.put(False)

    def stop_controller(self)->None:
        self.QueueStop.put(True)

    def resume_controller(self)->None:        
        self.QueueStop.put(False)
        
    def reset(self)->None: #go back to initial state
        self.ResetState = True

    def servo1state(self, state:str):
        match state:
            case "0":
                GPIO.output(self.SERVO1_PIN1, GPIO.LOW)
                GPIO.output(self.SERVO1_PIN2, GPIO.LOW)
            case "1":
                GPIO.output(self.SERVO1_PIN1, GPIO.LOW)
                GPIO.output(self.SERVO1_PIN2, GPIO.HIGH)
            case "2":
                GPIO.output(self.SERVO1_PIN1, GPIO.HIGH)
                GPIO.output(self.SERVO1_PIN2, GPIO.LOW)
            case "3":
                GPIO.output(self.SERVO1_PIN1, GPIO.HIGH)
                GPIO.output(self.SERVO1_PIN2, GPIO.HIGH)
                       
    def servo2state(self, state:bool):
        if state:
            GPIO.output(self.SERVO2_PIN, GPIO.HIGH)
        else:   
            GPIO.output(self.SERVO2_PIN, GPIO.HIGH)

    def statemachine(self, QueueShutdown, QueueCorrectionFromCV)->None:
        shutdown:bool = False
        current_state:str = ""
        x:int = 0.0
        delta_x:float = 0
        delta_v:int = 0
        
        self.SERVO1_PIN1 = 3
        self.SERVO1_PIN2 = 4
        self.SERVO2_PIN = 14
        GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.SERVO1_PIN1, GPIO.OUT)
        GPIO.setup(self.SERVO1_PIN2, GPIO.OUT)
        GPIO.setup(self.SERVO2_PIN, GPIO.OUT)
        _currentstate:str = ""
        _delta = None
        while True:
            self.state = current_state
            
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown state machine")              
                return
            if self.ResetState:
                current_state = "reset"
            if not QueueCorrectionFromCV.empty():
                detection, delta_x, delta_v = QueueCorrectionFromCV.get()  
            # if current_state != _currentstate:
            #     print(self.state, self.current_position_in_mm, delta_v)
            #     _currentstate = current_state
            if delta_v != _delta:
                print(self.state, self.current_position_in_mm, delta_v)
                _delta = delta_v

              
            match current_state:
                case "reset":
                    self.QueueSetPoint.put([1, 60 * self.speedFactor])
                    
                    if self.current_position_in_mm <=1:
                        current_state = "p1"
                case "p0":
                    self.QueueSetPoint.put([150, 60 * self.speedFactor])
                    
                    if self.current_position_in_mm < 159:
                        current_state = "p1"
                case "p1":
                    x = int(210/self.mm_per_step)
                    self.QueueSetPoint.put([x, 100 * self.speedFactor])
                    if self.current_position_in_mm > 209 :  
                        time.sleep(2)                      
                        current_state = "p2"
                case "p2":
                    x = int(215/self.mm_per_step)
                    self.QueueSetPoint.put([x, 10 * self.speedFactor])
                    if self.current_position_in_mm > 214 :  
                        time.sleep(2)                      
                        current_state = "p3"
                case "p3":
                    x = int(220/self.mm_per_step)
                    self.QueueSetPoint.put([x, 10 * self.speedFactor])
                    if self.current_position_in_mm > 219 :  
                        time.sleep(2)                      
                        current_state = "p4"
                case "p4":
                    x = int(225/self.mm_per_step)
                    self.QueueSetPoint.put([x, 10 * self.speedFactor])
                    if self.current_position_in_mm > 224 :  
                        time.sleep(2)                      
                        current_state = "p5"
                case "p5":
                    x = int(230/self.mm_per_step)
                    self.QueueSetPoint.put([x, 10 * self.speedFactor])
                    if self.current_position_in_mm > 229 :  
                        time.sleep(2)                      
                        current_state = "p6"
                case "p6":
                    x = int(235/self.mm_per_step)
                    self.QueueSetPoint.put([x, 10 * self.speedFactor])
                    if self.current_position_in_mm > 234 :  
                        time.sleep(2)                      
                        current_state = "p7"
                case "p6":
                    x = int(240/self.mm_per_step)
                    self.QueueSetPoint.put([x, 10 * self.speedFactor])
                    if self.current_position_in_mm > 239 :  
                        time.sleep(2)                      
                        current_state = "p7"
                case "p7":
                    x = int(245/self.mm_per_step)
                    self.QueueSetPoint.put([x, 10 * self.speedFactor])
                    if self.current_position_in_mm > 244 :  
                        time.sleep(2)                      
                        current_state = "p0"
                case "p8":
                    x = int(250/self.mm_per_step)
                    self.QueueSetPoint.put([x, 10 * self.speedFactor])
                    if self.current_position_in_mm > 249 :  
                        time.sleep(2)                      
                        current_state = "p9"

                case "p9":
                    x = int(255/self.mm_per_step)
                    self.QueueSetPoint.put([x, 10 * self.speedFactor])
                    if self.current_position_in_mm > 254 :  
                        time.sleep(2)                      
                        current_state = "p0"
                case _:
                   current_state = "reset"
            
            time.sleep(0.001)    
           
    
    def getCurrentState(self)->str:
        t = self.state + " " + str(self.current_position_in_mm)
        return t

        

    def camera(self, QueueShutdown, QueueCurrentFrame, QueueCorrectionFromCV)->None:
        #setup camera
        shutdown:bool = False

        picam2 = Picamera2()
        # config = picam2.create_preview_configuration(main={"format":'XRGB8888', "size":(720,480)}, controls={'FrameRate':120})
        # #config = picam2.create_preview_configuration()
        # picam2.configure(config)
        picam2.configure(picam2.create_preview_configuration(main={"format":'XRGB8888', "size":(720, 480)}, controls={'FrameRate':120}))
        #picam2.configure("video")
        framecounter:int = 0
        #picam2.start_preview(Preview.Null)
        picam2.start()
        # Load the template and convert to grayscale (if not already grayscale)
        template = cv2.imread('/home/admin/Conveyor/template_1.jpg', 0)

        # # Apply histogram equalization to the template 
        # template = cv2.equalizeHist(template)

        # Get the initial dimensions of the template
        original_w, original_h = template.shape[::-1]

        # Define scale factors to iterate over
        scale_factors = np.linspace(0.9, 1.2, 10)
        _bottom_right = []
        while True:
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown: 
                    print("shutdown capture")
                    picam2.close()
                    return
            
                        

            # Loop
            
            color_image = picam2.capture_array()
            # print(color_image.shape())

            # Convert color image to grayscale for processing
            image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
            # # Apply histogram equalization to the image
            # image = cv2.equalizeHist(image)

            # Initialize variables to store the best match
            best_match_val = -1
            best_match_loc = None
            best_match_scale = 1.0

                # Iterate over scale factors
            for scale in scale_factors:
                # Resize the template
                scaled_template = cv2.resize(template, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
                
                # Get dimensions of the scaled template
                w, h = scaled_template.shape[::-1]
                
                # Perform template matching
                result = cv2.matchTemplate(image, scaled_template, cv2.TM_CCOEFF_NORMED)
                
                # Find the best match location
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                
                # Update the best match if the current one is better
                if max_val > best_match_val:
                    best_match_val = max_val
                    best_match_loc = max_loc
                    best_match_scale = scale
                    best_match_size = (w, h)
                #print("best match value", best_match_val)
                # Draw a red rectangle around the best match on the color image
            if best_match_loc:
                top_left = best_match_loc
                bottom_right = (top_left[0] + best_match_size[0], top_left[1] + best_match_size[1])
                cv2.rectangle(color_image, top_left, bottom_right, (0, 0, 255), 1)
            
            # print(round(best_match_val,2), "matched. Bottom right at", bottom_right)
            # Display result
            cv2.imshow('Template Matching', color_image)
            cv2.waitKey(1)
            _bottom_right.append(bottom_right)
            if len(_bottom_right) == 3:
                _bottom_right.pop(0)
                if _bottom_right[0]==_bottom_right[1]:
                    QueueCorrectionFromCV.put([False, 0.0, bottom_right]) #[bool detection, float position, int speed]
       

            
           
            
       
    def convertCapturedImage(self, QueueShutdown, QueueCurrentFrame)->None:
        print("convert image")
        shutdown:bool = False
        while True:
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown convert image")              
                return
            if not QueueCurrentFrame.empty():
                frame = QueueCurrentFrame.get()
                while(not QueueCurrentFrame.empty()):
                    # print("converting")
                    try:
                        frame = QueueCurrentFrame.get()
                    except:
                        ...
                opencv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                cv2.imwrite("test.jpg", opencv_image)
                self.convertedImage = Image.fromarray(opencv_image)
            else:
                time.sleep(0.005)  
   

    def getCurrentImage(self):
        # print("get current image", self.convertedImage)
        return self.convertedImage
    
    def OnOff(self, OnOff:str)->None:
        if OnOff == "On":
            self.startup()
        elif OnOff == "Off":
            self.shutdown()
        
    def startup(self)->None:
        
        process = Process(target=self.driver, args=(self.QueueShutdown, self.QueueStop, self.QueueSetPoint, self.QueueCurrentPosition,))      
        self.process_list.append(process)     

        process = Thread(target=self.current_position_converter, args=(self.QueueShutdown, self.QueueCurrentPosition))
        self.process_list.append(process)

        process = Process(target=self.camera, args=(self.QueueShutdown, self.QueueCurrentFrame, self.QueueCorrectionFromCV,))
        self.process_list.append(process)  

        # process = Thread(target=self.convertCapturedImage, args=(self.QueueShutdown, self.QueueCurrentFrame,))
        # self.process_list.append(process) 

        process = Thread(target=self.statemachine, args=(self.QueueShutdown, self.QueueCorrectionFromCV,))
        self.process_list.append(process)
        
        for process in self.process_list:
            process.start()

    def shutdown(self)->None:
        self.QueueShutdown.put(True)
        self.QueueShutdown.put(True)
        self.QueueShutdown.put(True)
        self.QueueShutdown.put(True)
        self.QueueShutdown.put(True)
        self.QueueShutdown.put(True)
        self.QueueShutdown.put(True)
        self.QueueShutdown.put(True)

        #for process in self.process_list:
        #    process.join()

        while (not self.QueueShutdown.empty()):
                self.QueueShutdown.get()

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
    
    def setMode(self, mode:str)->None:
        match mode:
            case "continues":
                self.continuesMode = True
            case "single":
                self.continuesMode = False

    def setSpeed(self, speed: float)->None:
        self.speedFactor = max(min(speed, 1), 0)

def main()->None:   
    ctrl = governor()
    ctrl.OnOff("On")
    ctrl.start_controller()

if __name__ == "__main__":
    main()
