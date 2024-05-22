from threading import Thread
import RPi.GPIO as GPIO
import time
from multiprocessing import Process, Queue
import cv2
from PIL import Image
from picamera2 import Picamera2
from mv_shrimp import find_shrimp_features



class governor():
    def __init__(self):            
        self.process_list = []
        self.QueueShutdown = Queue()
        self.QueueStop = Queue()
        self.QueueSetPoint = Queue()
        self.QueueCurrentPosition = Queue()
        self.QueueCurrentFrame = Queue()
        self.QueueCorrectionFromCV = Queue()
        self.QueueDetectionMode = Queue()
        self.QueueContinuousMode = Queue()
        self.QueueSolenoidState = Queue()
        self.current_position_in_mm:float = 0.0
        self.convertedImage = None
        self.ResetState:bool = False
        self.state:str = "reset"
        self.speedFactor:float = 1
        # self.continuousMode:bool = False
       

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
        current_position:int = 3000
        position_setpoint:int = 3000 
        # setup GPIO
        # Define GPIO pins
        self.STEP_PIN = 18
        self.DIR_PIN = 27
        self.LL_PIN = 6

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.LL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        current_speed:int = 0
        position_error:float = 0.0
        speed_setpoint:int = 0
        QueueCurrentPosition.put(1000)
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
                while (not QueueSetpoint.empty()):
                    try:
                        position_setpoint, speed_setpoint = QueueSetpoint.get()
                    except:
                        ...

            # print("driver", position_setpoint, speed_setpoint, current_position)

            # position_error = position_setpoint - current_position     
            if GPIO.input(self.LL_PIN)==0 and position_error<0:
                # stop = True
                current_position = 0

            if not stop:              
                
                # simple controller
                position_error = position_setpoint - current_position                   
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
                # print(self.current_position_in_mm)
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
        print("start_controller")
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
            case _:
                GPIO.output(self.SERVO1_PIN1, GPIO.LOW)
                GPIO.output(self.SERVO1_PIN2, GPIO.LOW)
                       
    def servo2state(self, state:str):
        match state:
            case "open":
                GPIO.output(self.SERVO2_PIN, GPIO.HIGH)
            case "close":   
                GPIO.output(self.SERVO2_PIN, GPIO.LOW)
            case _:
                GPIO.output(self.SERVO2_PIN, GPIO.LOW)
                
    def solenoidstate(self, QueueShutdown, QueueSolenoidState):
        print("start solenoidstate")
        self.Solenoid_PIN = 22
        GPIO.setmode(GPIO.BCM)   
        GPIO.setup(self.Solenoid_PIN, GPIO.OUT)
        state = "close"
        shutdown = False
        while True:
            if not QueueShutdown.empty():
                    shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown driver")              
                return
            if not QueueSolenoidState.empty():
                state = QueueSolenoidState.get()
                match state:
                    case "open":
                        GPIO.output(self.Solenoid_PIN, GPIO.HIGH)
                    case "close":
                        GPIO.output(self.Solenoid_PIN, GPIO.LOW)
                    case "pulse":
                        GPIO.output(self.Solenoid_PIN, GPIO.HIGH)
                        time.sleep(0.1)
                        GPIO.output(self.Solenoid_PIN, GPIO.LOW)
                        state = "close"
                    case _:
                        GPIO.output(self.Solenoid_PIN, GPIO.LOW)
            else:
                time.sleep(0.01)

    def statemachine(self, QueueShutdown, QueueCorrectionFromCV, QueueContinuousMode, QueueSolenoidState)->None:
        shutdown:bool = False
        current_state:str = "reset"
        x:int = 0.0
        delta_x:float = 0
        delta_v:int = 0
        detection:bool = False
        self.SERVO1_PIN1 = 17
        self.SERVO1_PIN2 = 4
        self.SERVO2_PIN = 14
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.SERVO1_PIN1, GPIO.OUT)
        GPIO.setup(self.SERVO1_PIN2, GPIO.OUT)
        GPIO.setup(self.SERVO2_PIN, GPIO.OUT)
        
        continuousMode = False 
            
        while True:
            self.state = current_state
            #print(self.state)
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if not QueueContinuousMode.empty():
                continuousMode = QueueContinuousMode.get()
                print("QueueMode", continuousMode)
            if shutdown:  
                print("shutdown state machine")              
                return
            if self.ResetState:
                current_state = "reset"
            if not QueueCorrectionFromCV.empty():
                detection, delta_x= QueueCorrectionFromCV.get()   
            # print(current_state) 
            match current_state:
                case "reset":
                    # self.QueueStop.put(False) 
                    # print("reset")
                    # if not self.QueueSetPoint.empty():
                    #     while (not self.QueueSetPoint.empty()):
                    #         try:
                    #             self.QueueSetPoint.get()
                    #         except:
                    #             ...
                    
                    self.QueueSetPoint.put([0, 60])
                    self.QueueDetectionMode.put("Off")
                    self.servo1state("0")
                    self.servo2state("open")
                    if self.current_position_in_mm <= 0.1:
                        time.sleep(0.2)
                        current_state = "start"
                        # print(self.current_position_in_mm)
                    
                case "start":
                    # print("start", self.current_position_in_mm)
                    # time.sleep(1)
                    x = int(302/self.mm_per_step)
                    self.QueueSetPoint.put([x, 400 * self.speedFactor])
                    self.QueueDetectionMode.put("Off")
                    self.servo1state("0")
                    self.servo2state("open")
                    if self.current_position_in_mm >10:
                        current_state = "synchronize speed"
                case "synchronize speed":
                    x = int(342/self.mm_per_step)
                    self.QueueSetPoint.put([x, 400 * self.speedFactor])
                    self.QueueDetectionMode.put("Off")
                    self.servo1state("0")
                    self.servo2state("close")
                    if self.current_position_in_mm > 30: 
                        QueueSolenoidState.put("pulse")                    
                        current_state = "pickup shrimp"
                case "pickup shrimp":
                    x = int(342/self.mm_per_step)
                    self.QueueSetPoint.put([x, 400 * self.speedFactor])
                    self.QueueDetectionMode.put("Off")
                    self.servo1state("1")
                    self.servo2state("Close")                    
                    if self.current_position_in_mm > 150:                      
                        # current_state = "analyze shrimp"
                        current_state = "move shrimp forward"
                case "move shrimp forward":
                    x = int(1500/self.mm_per_step)
                    self.QueueSetPoint.put([x, 250 * self.speedFactor])
                    self.QueueDetectionMode.put("Nose")
                    self.servo1state("1")
                    self.servo2state("Close")
                    _current_position=0.0
                    if detection:
                        # _current_position = self.current_position_in_mm
                      
                        # x = int((_current_position + 10)/self.mm_per_step)
                        current_state = "move shrimp to camera"
                        # print("move shrimp to camera", delta_x)
                    if self.current_position_in_mm > 1490:
                        current_state = "No shrimp"
                case "No shrimp":
                    self.QueueStop.put(True)                
                    time.sleep(0.1)
                case "move shrimp to camera":      
                    self.QueueDetectionMode.put("Ear") 
                    x = int(1500/self.mm_per_step)
                        # print("found shrimp", _current_position, self.current_position_in_mm, _current_position/self.mm_per_step, x)
                        # self.QueueStop.put(True) 
                    self.QueueSetPoint.put([x, 5 * self.speedFactor])
                        # self.QueueStop.put(True)
                    self.servo1state("1")
                    self.servo2state("Close")
                    print("move to camera", delta_x)
                    if delta_x < 2:
                       
                        current_state = "analyze shrimp"

                case "analyze shrimp":
                    self.servo1state("1")
                    self.servo2state("Close")
                    print("delta", delta_x) 
                    if abs(delta_x)<0.9:
                        current_state = "belly cut"
                        self.QueueStop.put(True) 
                    else:    
                        if delta_x < 0:
                            if not self.QueueSetPoint.empty():
                                x = int((self.current_position_in_mm-0.1)/self.mm_per_step)                    
                                self.QueueSetPoint.put([x, 2 * self.speedFactor])
                            
                        if delta_x > 0:
                            if not self.QueueSetPoint.empty():
                                x = int((self.current_position_in_mm+0.1)/self.mm_per_step)                    
                                self.QueueSetPoint.put([x, 2 * self.speedFactor])
                        
                    
                case "belly cut":
                    self.servo1state("2")
                    self.servo2state("Close")
                    self.QueueDetectionMode.put("Off")
                    time.sleep(1)
                    current_state = "drop off"
                case "drop off":
                    self.servo1state("0")
                    time.sleep(1)
                    self.servo2state("open")
                    self.QueueDetectionMode.put("Off")
                    time.sleep(1)
                    current_state = "return to start"
                case "return to start":
                    self.servo1state("0")
                    self.servo2state("open")
                    self.QueueStop.put(False) 
                    self.QueueDetectionMode.put("Off")
                    self.QueueSetPoint.put([0, 600 * self.speedFactor])                   
                    if self.current_position_in_mm <=1:
                        current_state = "continue"
                case "continue":
                    # print("Mode:", continuousMode)
                    if continuousMode:
                        # self.QueueSetPoint.put([0, 60 * self.speedFactor])
                        current_state = "reset"
                        self.QueueStop.put(False)                        
                    else:
                        current_state = "continue"
                        
                        self.QueueStop.put(True) 
                        time.sleep(0.1)
                case _:
                   current_state = "reset"
            
            time.sleep(0.008)    
           
    
    def getCurrentState(self)->str:
        t = self.state + " " + str(self.current_position_in_mm)
        return t

        

    def camera(self, QueueShutdown, QueueCurrentFrame, QueueCorrectionFromCV, QueueDetectionMode)->None:
        #setup camera
        shutdown:bool = False

        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"format":'XRGB8888', "size":(720,480)}, controls={'FrameRate':120})
        #config = picam2.create_preview_configuration()
        picam2.configure(config)
        #picam2.configure("video")
        framecounter:int = 0
        #picam2.start_preview(Preview.Null)
        detectionMode:str=""

        picam2.start()
        shrimp = find_shrimp_features()
        shrimp.set_ear_template('/home/admin/Conveyor/template1.jpg',low_scale_factor=0.9, high_scale_factor=1.1, scale_step=3)
        nose_dark_threshold = 100  # Settable dark value threshold
        nose_area_threshold = 100  # Minimum area of the detected object to be considered valid
        intercept:float = -2923.5706
        slope:float = 14.9564
        while True:
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown: 
                print("shutdown capture")
                picam2.close()
                return
            if not QueueDetectionMode.empty():
                detectionMode = QueueDetectionMode.get()
                while (not QueueDetectionMode.empty()):
                    try:
                        detectionMode = QueueDetectionMode.get()
                    except:
                        ...
                
            frame = picam2.capture_array()  
            height, width = frame.shape[:2]
            match detectionMode:
                case "Off":
                    # frame = picam2.capture_array()  
                    QueueCorrectionFromCV.put([False, 100.0]) #[bool detection, float position, int speed]
                case "Nose":
                
                    # frame = picam2.capture_array()                  
                    # height, width = frame.shape[:2] 
                    # # Define ROI: (x1, y1, x2, y2)
                    nose_roi = (0, int(0.3 * height), int(0.5 * width), int(0.7 * height)) 
                    found_nose, topleft, lowerright = shrimp.find_nose(frame, nose_roi, nose_dark_threshold, nose_area_threshold)
                    if found_nose:   
                        cv2.rectangle(frame, topleft, lowerright, (0, 0, 255), 1)   
                        QueueCorrectionFromCV.put([found_nose, 100.0]) #[bool detection, float position, int speed]
                case "Ear":
                    # frame = picam2.capture_array()
                    # print(frame)
                    # time.sleep(10)
                    #ret, frame = cap.read()
                        
                   
                    ear_roi = (0, int(0.3 * height), int(0.7 * width), int(0.7 * height))
                    found_ear, topleft, lowerright = shrimp.find_ear(frame, ear_roi, match_threshold=0.8)  
                    if found_ear:    
                        cv2.rectangle(frame, topleft, lowerright, (0, 255, 0), 1)
                        blade_mm=(480-intercept)/slope
                        distance_mm=(lowerright[0]-intercept)/slope
                        difference = blade_mm - distance_mm - 5.145741756037534
                        # print(difference)
                        QueueCorrectionFromCV.put([found_ear, difference]) #[bool detection, float position, int speed]
                
                case _:
                    
                    QueueCorrectionFromCV.put([False, 100.0]) #[bool detection, float position, int speed]
           
            cv2.line(frame, [400,0], [400,480], (255, 0, 0), 2)
            framecounter += 1
            if framecounter == 20:
                framecounter = 0
                QueueCurrentFrame.put(frame)

            
            
            
       
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
        # print("OnOff", OnOff)
        if OnOff == "On":
            self.startup()
        elif OnOff == "Off":
            self.shutdown()
        
    def startup(self)->None:
        
        process = Process(target=self.driver, args=(self.QueueShutdown, self.QueueStop, self.QueueSetPoint, self.QueueCurrentPosition,))      
        self.process_list.append(process)     

        process = Thread(target=self.current_position_converter, args=(self.QueueShutdown, self.QueueCurrentPosition))
        self.process_list.append(process)

        process = Process(target=self.camera, args=(self.QueueShutdown, self.QueueCurrentFrame, self.QueueCorrectionFromCV, self.QueueDetectionMode,))
        self.process_list.append(process)  

        process = Thread(target=self.convertCapturedImage, args=(self.QueueShutdown, self.QueueCurrentFrame,))
        self.process_list.append(process) 

        process = Thread(target=self.statemachine, args=(self.QueueShutdown, self.QueueCorrectionFromCV, self.QueueContinuousMode, self.QueueSolenoidState,))
        self.process_list.append(process)

        process = Thread(target=self.solenoidstate, args=(self.QueueShutdown, self.QueueSolenoidState,))
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
        # print("set", mode)
        match mode:
            case "continuous":
                self.QueueContinuousMode.put(True)
            case "single":
                self.QueueContinuousMode.put(False)

    def setSpeed(self, speed: float)->None:
        self.speedFactor = max(min(speed, 1), 0)

def main()->None:   
    ctrl = governor()
    ctrl.OnOff("On")
    ctrl.start_controller()

if __name__ == "__main__":
    main()
