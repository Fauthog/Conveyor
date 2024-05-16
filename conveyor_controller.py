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
        STEP_PIN = 18
        DIR_PIN = 27
        LL_PIN = 3

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(STEP_PIN, GPIO.OUT)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        GPIO.setup(LL_PIN, GPIO.IN)

        current_speed:int = 0
        position_error:float = 0.0
        speed_setpoint:int = 0
        
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

            position_error = position_setpoint - current_position     
            if GPIO.input(LL_PIN) and position_error<0:
                stop = True
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

    def statemachine(self, QueueShutdown, QueueCorrectionFromCV)->None:
        shutdown:bool = False
        current_state:str = ""
        x:int = 0.0
        delta_x:float = 0
        delta_v:int = 0
        
        SERVO_PIN = 5
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)

        while True:
            self.state = current_state
            #print(self.state)
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown state machine")              
                return
            if self.ResetState:
                current_state = "reset"
            if not QueueCorrectionFromCV.empty():
                detection, delta_x, delta_v = QueueCorrectionFromCV.get()    
            match current_state:
                case "reset":
                    self.QueueSetPoint.put([1, 60 * self.speedFactor])
                    GPIO.output(SERVO_PIN, GPIO.LOW)
                    if self.current_position_in_mm <=1:
                        current_state = "start"
                case "start":
                    x = int(302/self.mm_per_step)
                    self.QueueSetPoint.put([x, 400 * self.speedFactor])
                    GPIO.output(SERVO_PIN, GPIO.LOW)
                    if self.current_position_in_mm >10:
                        current_state = "synchronize speed"
                case "synchronize speed":
                    x = int(342/self.mm_per_step)
                    self.QueueSetPoint.put([x, 400 * self.speedFactor])
                    GPIO.output(SERVO_PIN, GPIO.HIGH)
                    if self.current_position_in_mm > 30:                        
                        current_state = "pickup shrimp"
                case "pickup shrimp":
                    x = int(342/self.mm_per_step)
                    self.QueueSetPoint.put([x, 400 * self.speedFactor])
                    GPIO.output(SERVO_PIN, GPIO.LOW)
                    if self.current_position_in_mm > 60:                       
                        # current_state = "analyze shrimp"
                        current_state = "move shrimp forward"
                case "move shrimp forward":
                    x = int(1500/self.mm_per_step)
                    self.QueueSetPoint.put([x, 400 * self.speedFactor])
                    if detection:
                        current_state = "move shrimp to camera"
                    if self.current_position_in_mm > 1490:
                        current_state = "No shrimp"
                    GPIO.output(SERVO_PIN, GPIO.LOW)
                case "No shrimp":
                    self.QueueStop.put(True)
                    time.sleep(0.1)
                case "move shrimp to camera":
                    ...
                case "analyze shrimp":
                    x = int(342/self.mm_per_step)
                    self.QueueSetPoint.put([x, (200 * self.speedFactor) + delta_v])
                    GPIO.output(SERVO_PIN, GPIO.LOW)
                    if self.current_position_in_mm > 250:                       
                        current_state = "belly cut"
                case "belly cut":
                    x = int(342/self.mm_per_step)
                    self.QueueSetPoint.put([x, 200 * self.speedFactor])
                    GPIO.output(SERVO_PIN, GPIO.LOW)
                    if self.current_position_in_mm > 300:                       
                        current_state = "drop off"
                case "drop off":
                    x = int((342 + delta_x)/self.mm_per_step)
                    self.QueueSetPoint.put([x, 300 * self.speedFactor]) 
                    GPIO.output(SERVO_PIN, GPIO.LOW)
                    if self.current_position_in_mm > 340:                       
                        current_state = "return to start"
                case "return to start":
                    self.QueueSetPoint.put([1, 600 * self.speedFactor])
                    GPIO.output(SERVO_PIN, GPIO.LOW)
                    if self.current_position_in_mm <=1:
                        current_state = "continue"
                case "continue":
                    if self.continuesMode:
                        current_state = "start"
                    else:
                        current_state = "continue"
                        time.sleep(0.1)
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
        config = picam2.create_preview_configuration(main={"format":'XRGB8888', "size":(720,480)}, controls={'FrameRate':120})
        #config = picam2.create_preview_configuration()
        picam2.configure(config)
        #picam2.configure("video")
        framecounter:int = 0
        #picam2.start_preview(Preview.Null)
        picam2.start()
        while True:
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown: 
                    print("shutdown capture")
                    picam2.close()
                    return
            
            # Read the image as color
            color_image = picam2.capture_array()
           
          
            # Define the region of interest (ROI) - rows 100 to 360
            roi = color_image[100:360, :]
            
            # Convert the ROI to grayscale
            gray_image = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur to the grayscale ROI
            blurred_image = cv2.GaussianBlur(gray_image, (3, 3), 0)
            
            # Invert the blurred grayscale image
            inverted_gray_image = 255 - blurred_image
            
            # Apply binary thresholding to the inverted image
            _, thresh_image = cv2.threshold(inverted_gray_image, 127, 255, cv2.THRESH_BINARY)
            
            # Find all contours in the thresholded image
            contours, _ = cv2.findContours(thresh_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
            # Print the number of contours detected
            # print(f"Number of contours detected: {len(contours)}")
            
            # Draw contours on the original color ROI
            result = roi.copy()
            cv2.drawContours(result, contours, -1, (0, 255, 0), 2)
            
            # Find the rightmost contour
            rightmost_contour = max(contours, key=lambda c: cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2])
            
            # Get the rightmost pixel in the rightmost contour
            rightmost_point = tuple(rightmost_contour[rightmost_contour[:,:,0].argmax()][0])
            
            # Print the location of the rightmost pixel
            # print(f"Location of the rightmost pixel: {rightmost_point}")
            
            # Visualize the rightmost pixel with a small red vertical line (50 pixels long)
            result_with_point = result.copy()
            cv2.line(result_with_point, rightmost_point, (rightmost_point[0], rightmost_point[1] - 50), (0, 0, 255), 2)
            
            # Replace the ROI in the original image with the result
            output_image = color_image.copy()
            output_image[100:360, :] = result_with_point
            # cv2.imwrite("test.jpg", output_image)

            framecounter += 1
            if framecounter == 20:
                framecounter = 0
                QueueCurrentFrame.put(output_image)

            
            QueueCorrectionFromCV.put([False, 0.0, 0]) #[bool detection, float position, int speed]
            
       
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

        process = Thread(target=self.convertCapturedImage, args=(self.QueueShutdown, self.QueueCurrentFrame,))
        self.process_list.append(process) 

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
