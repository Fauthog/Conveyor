from threading import Thread
import RPi.GPIO as GPIO
import time
import cv2
from picamera2 import Picamera2
from mv_shrimp2 import find_shrimp_features




class governor():
    def __init__(self):      
        self.current_position_in_pulses:int = 0
        self.goto_position:float = 0
        self.process_list = []
        self.data = []

        # magic numbers start
        self.steps_per_rev:int = 400
        self.length:float = 350.0
        self.steps:int = 1473
        self.x_min:int = 0
        self.x_max:int = self.steps
        self.mm_per_step:float = self.length / self.steps
        # magic numbers end

        process = Thread(target=self.camera)
        self.process_list.append(process)

        process = Thread(target=self.driver)
        self.process_list.append(process)

        for process in self.process_list:
            process.start()

    def camera(self)->None:
        picam2 = Picamera2()
        config = picam2.create_preview_configuration(main={"format":'XRGB8888', "size":(720,480)}, controls={'FrameRate':120})
        picam2.configure(config)
        picam2.start()
        shrimp = find_shrimp_features()
        nose_dark_threshold = 100  # Settable dark value threshold
        nose_area_threshold = 100  # Minimum area of the detected object to be considered valid
        intercept:float = -2923.5706
        slope:float = 14.9564
        _position:float = 0
        _x = None
        # cv2.namedWindow('Shrimp', cv2.WINDOW_NORMAL) 
        while True:
            frame = picam2.capture_array()
            height, width = frame.shape[:2]
            # nose_roi = (0, int(0.3 * height), int(0.5 * width), int(0.7 * height)) 
            nose_roi = (0, 0, width, height) 
            found_nose, x, y = shrimp.find_chessboard_tl_corner(frame, 4, 3)
            if found_nose:
                
                if self.current_position_in_pulses > 100 and self.current_position_in_pulses * self.mm_per_step != _position and x != _x:
                    # print(self.current_position_in_pulses * self.mm_per_step, "mm", lowerright[0])
                    data = [self.current_position_in_pulses * self.mm_per_step, x]
                    self.data.append(data)        
            # Display the frame
            cv2.imshow('Frame', frame)
            
            # Break the loop on 'q' key press
            cv2.waitKey(1)
    
    def print_data(self)->None:
        print("data [mm, pixel coordinate lower right]")
        for data in self.data:
            print(data)
        print("/data")

    def Position0(self)->None:
        self.STEP_PIN = 18
        self.DIR_PIN = 27
        self.LL_PIN = 6
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.LL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        while GPIO.input(self.LL_PIN)==1:
                    # print("0")
                    GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * 60)
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    self.high_precision_sleep(delay)                                       
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    self.high_precision_sleep(delay)            
    
        self.current_position_in_pulses:int = 0            


    def driver(self):
        self.speed:int = 5
        self.STEP_PIN = 18
        self.DIR_PIN = 27
        self.LL_PIN = 6
       
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.LL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.Position0()
        while True:
            
            self.current_position_in_mm = self.current_position_in_pulses * self.mm_per_step
            if self.current_position_in_mm < 200:
                self.speed = 25
            else:
                self.speed = 1
            position_error = self.goto_position - self.current_position_in_mm
            
            if abs(position_error) < 1:
                self.goto()
            else:
                if position_error < 0:
                    GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * self.speed)
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    self.high_precision_sleep(delay)                                       
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    self.high_precision_sleep(delay)
                    self.current_position_in_pulses -= 1
                
                else:
                    GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * self.speed)
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    self.high_precision_sleep(delay)                                       
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    self.high_precision_sleep(delay)
                    self.current_position_in_pulses += 1


    def goto(self)->None:
        user_input = input("Goto mm:")
        if user_input[0] == "e":
            self.print_data()
        else:
            self.goto_position=min(max(float(user_input),0),350)
        print("going to ", self.goto_position)

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
    ctrl = governor()
    #ctrl.Position0()
    

if __name__ == "__main__":
    main()
