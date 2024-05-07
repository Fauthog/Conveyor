from pynput.keyboard import Key, Listener
from threading import Thread
import RPi.GPIO as GPIO
import time
from multiprocessing import Process, Queue
import cv2
from PIL import Image
from picamera2 import Picamera2, Preview



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

        self.startup()


    def driver(self, QueueShutdown, QueueStop, QueueSetpoint, QueueCurrentPosition)->None:
        print("driver")
        shutdown:bool = False
        stop:bool = True
        current_position:int = 0
        # setup GPIO
        # Define GPIO pins
        self.STEP_PIN = 18
        self.DIR_PIN = 27

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)

        current_speed:int = 0
        position_error:float = 0.0
        speed_setpoint:int = 0
        
        while (True):
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown driver")              
                return

            if not QueueStop.empty():
                stop = QueueStop.get()
            if not QueueSetpoint.empty():
                position_setpoint, speed_setpoint = QueueSetpoint.get()

            if not stop:              
                
                # simple controller
                position_error = position_setpoint - current_position                   
                error_abs = abs(position_error)
                
                if 50 <= error_abs <=100:
                    speed_setpoint = min(self.medium_speed, self.speed_setpoint)
                if error_abs <= 10:
                    speed_setpoint = min(self.low_speed, self.speed_setpoint)
                if error_abs < 1:
                    self.active = False
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

                print("driver", position_error, limited_speed)
                lower_limit_lock:bool = (self.current_position == self.x_min) and (limited_speed < 0)
                upper_limit_lock:bool = (self.current_position == self.x_max) and (limited_speed > 0)
                if not lower_limit_lock or upper_limit_lock:
                    if limited_speed < 0:
                        GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                        delay:float = 60.0 / (self.steps_per_rev * abs(limited_speed))
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        self.high_precision_sleep(delay)
                        current_position -= 1
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                        self.high_precision_sleep(delay)

                    elif limited_speed > 0:
                        GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                        delay:float = 0.5 * 60.0 / (self.steps_per_rev * limited_speed)
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        self.high_precision_sleep(delay)
                        current_position +=1
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                        self.high_precision_sleep(delay)
                    
                    current_speed = limited_speed
                    QueueCurrentPosition.put(current_position)                

            else:
                # ramp down to stop
                if current_speed is not 0:                    
                    if limited_speed > 0:
                        limited_speed = max(current_speed-60, 0) 
                        GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                        delay:float = 0.5 * 60.0 / (self.steps_per_rev * limited_speed)
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        self.high_precision_sleep(delay)
                        current_position +=1
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                        self.high_precision_sleep(delay)
                        current_speed = limited_speed

                    elif limited_speed < 0:
                        limited_speed = min(current_speed+60, 0)
                        GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                        delay:float = 60.0 / (self.steps_per_rev * abs(limited_speed))
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                        self.high_precision_sleep(delay)
                        current_position -= 1
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                        self.high_precision_sleep(delay)
                        current_speed = limited_speed
                else:
                    # if we are stopped, we wait for stop:bool to be false
                    time.sleep(0.05)
				
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
                while(not QueueCurrentPosition.empty()):                   
                    try:
                        position = QueueCurrentPosition.get()
                    except:
                        ...
                self.current_position_in_mm = position * self.mm_per_step
            else:
                time.sleep(0.005)

    def get_current_position(self)->float:
        print("get_current_position")
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
        self.QueueStop.put(False)

    def stop_controller(self)->None:
        self.QueueStop.put(True)

    def resume_controller(self)->None:        
        self.QueueStop.put(False)
        
    def reset(self)->None: #go back to initial state
        self.ResetState = True

    def statemachine(self, QueueShutdown)->None:
        shutdown:bool = False
        current_state:str = ""
        while True:
            self.state = current_state
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown state machine")              
                return
            if self.ResetState:
                current_state = "reset"
            
            match current_state:
                case "reset":
                    self.QueueSetPoint.put([0, 60])
                    if self.current_position_in_mm <=0.1:
                        current_state = "start"
                case "start":
                    self.QueueSetPoint.put([0, 60])
                    if self.current_position_in_mm >10:
                        current_state = "synchronize speed"
                case "synchronize speed":
                    self.QueueSetPoint.put([300, 400])
                    if self.current_position_in_mm > 100:                        
                        current_state = "pickup shrimp"
                case "pickup shrimp":
                    self.QueueSetPoint.put([300, 400])
                    if self.current_position_in_mm > 150:                       
                        current_state = "analyze shrimp"
                case "analyze shrimp":
                    self.QueueSetPoint.put([300, 200])
                    if self.current_position_in_mm > 200:                       
                        current_state = "belly cut"
                case "belly cut":
                    self.QueueSetPoint.put([300, 200])
                    if self.current_position_in_mm > 250:                       
                        current_state = "drop off"
                case "drop off":
                    self.QueueSetPoint.put([300, 300]) 
                    if self.current_position_in_mm > 300:                       
                        current_state = "return to start"
                case "return to start":
                    self.QueueSetPoint.put([0, 400])
                    if self.current_position_in_mm <=0.1:
                        current_state = "start"                     
                case _:
                   current_state = "reset"
            time.sleep(0.001)    
           
    
    def getCurrentState(self)->str:
        return self.state      

        

    def camera(self, QueueShutdown, QueueCurrentFrame, QueueCorrectionFromCV)->None:
        #setup camera
        shutdown:bool = False

        picam2 = Picamera2()
        # config = picam.create_preview_configuration(main={"format":'XRGB8888', "size":(720,480)}, controls={'FrameRate':120})
        config = picam2.create_preview_configuration()
        picam2.configure(config)
        # picam.configure("video")

        picam2.start_preview(Preview.Null)
        picam2.start()
        while True:
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown: 
                    print("shutdown capture")
                    picam2.close()
                    return
            
            img = picam2.capture_array()
            QueueCurrentFrame.put(img)

            
            QueueCorrectionFromCV.put([0.0, 0.0]) #[float position, float speed]
            
       
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
                opencv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
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

        process = Thread(target=self.convertCapturedImage, args=(self.QueueShutdown, self.QueueCurrentFrame,))
        self.process_list.append(process) 

        process = Thread(target=self.statemachine, args=(self.QueueShutdown,))
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

        for process in self.process_list:
            process.join()

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



# def main()->None:   


# if __name__ == "__main__":
#     main()
