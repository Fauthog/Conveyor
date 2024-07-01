from threading import Thread
import time
from multiprocessing import Process, Queue
import cv2
from PIL import Image
from picamera2 import Picamera2
from mv_shrimp import find_shrimp_features
import serial
from alpha5 import Alpha5Client
 

class driver():

    def __init__(self, ): 
        self.arduinoPort = "/dev/ttyUSB2"      
        self.setupSerialToArduino()
        self.setupAlpha()

    def setupAlpha(self)->None:
        self.alpha5 = Alpha5Client("/dev/ttyUSB0")
        self.alpha5.create_client()
        self.alpha5.connect()
        self.alpha5.set_show_query_response(False)

    def setupSerialToArduino(self)->None:
        self.arduino = serial.Serial(port=self.arduinoPort, baudrate=9600, timeout=2)
        # print(self.arduino.readline())
        # print(self.arduino.readline())
    
    def writeToArduino(self, bigServo:int, smallServo:int, solenoid:int)->None:
        if not self.arduino.is_open:
                self.arduino.open()
        cmd = str(bigServo) + "," + str(smallServo) + "," + str(solenoid)
        # print("cmd:", cmd)

        self.arduino.write((cmd + "\r\n").encode('utf-8'))

        # print(self.arduino.readline())
        # print(self.arduino.readline())
        # print()

    def cleanUpArduinoConnection(self)->None:
        if self.arduino !="None":           
            self.arduino.flushInput()
            self.arduino.flushOutput()
            self.arduino.close()   
        
    def Actuators(self, bigServo:int, smallServo:int, solenoid:int)->None:
        self.writeToArduino(bigServo, smallServo, solenoid)  

    def homing(self)->None:
        print('Set [S-ON] to ON')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='ON') # [S-ON]->CONT9
    
        print('Issue homing command')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=2, state='ON') # [ORG]->CONT11

        print("Homing...")
        print('Wait for the operation to complete')
        self.alpha5.wait_operation_complete(id=1)
        print('done!')        
     
        print('Set [S-ON] to OFF')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='OFF') # [S-ON]->CONT9

    def immediateOperation(self, Units:int, Speed:int)->None:
        # print('Set [S-ON] to ON')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='ON') # [S-ON]->CONT9

        # print('Send immediate operation setting')
        self.alpha5.send_immediate_operation_setting(id=1, units=Units, speed=Speed)
        # print('Set [START] to ON')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=1, state='ON') # [START]->CONT10
        # print('Set [START] to OFF')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=1, state='OFF') # [START]->CONT10


        # print('Wait for the operation to complete')
        self.alpha5.wait_operation_complete(id=1)


        # print('Set [S-ON] to OFF')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='OFF') # [S-ON]->CONT9
    

class statemachine():
    def __init__(self, driver, camera):
        print("init state machine") 
        self.driver = driver
        self.camera = camera
        self.state0:str = "init"
        self.state0Last:str = ""
        self.state1:str = "init"
        self.state1Last:str = ""        
        self.Start:bool = False
        self.cycling:bool = False
        self.resume:bool = False
        self.shutDown:bool = False
        self.hold:bool = False
        self.process_list = []
        self.error_state:str = ""
        self.homeIsSet:bool = False
        

        process = Thread(target=self.statemachine)
        self.process_list.append(process)

        for process in self.process_list:
            process.start()
    
    def homing(self):
        self.driver.Actuators(1500, 1250, 0)
        time.sleep(1)
        self.driver.homing()
        self.homeIsSet = True
    
    def shutdown(self):
        self.shutDown = True

    def printState(self):
        if self.state0 != self.state0Last:
            print("state0: ", self.state0)
            self.state0Last = self.state0[:]
        if self.state1 != self.state1Last:
            print("state1: ", self.state1)
            self.state1Last = self.state1[:]

    def getStates(self)->[]:
        return [self.state0, self.state1, self.error_state] 

    def setCycling(self, cycling:bool)->None:
        self.cycling = cycling

    def stop(self, hold:bool)->None:
        self.hold=hold
        

    def start(self):
        # print("state machine start")
        self.Start = True

    def reset(self)->None:
        self.state0 = "init"
        self.state1 = "init"
        self.error_state = ""
        self.Start = False
 
    def statemachine(self):
       

        while True:
            if self.shutDown:
                print("shutting down statemachine")
                return
            # print("state", self.state0, self.hold, self.Start)
            # print("isInit", self.driver.isInitPosition() )
            # self.printState()
            if not self.hold and self.homeIsSet:
                match self.state0:                
                    case "init":
                        self.driver.Actuators(1500, 1250, 0)
                        time.sleep(1)
                        self.driver.immediateOperation(30000, 150000)
                        self.state0="loop"
                        

                    case "loop":   
                            self.driver.Actuators(2000, 800, 0)                    
                            if self.cycling:
                                self.Start = True
                            else:
                                self.Start = False
                            self.state0 = "wait"
                    
                    case "wait":                        
                        if self.Start:
                            self.state0 = "start"   
                                                

                    case "start":
                        # time.sleep(2)
                        self.driver.Actuators(2000, 800, 0)
                        # time.sleep(0.1)
                        self.state0 = "synchronize"
                        

                    case "synchronize":
                        self.driver.Actuators(2000, 1250, 0)
                        # time.sleep(0.1)                    
                        self.state0 = "pickup"
                    
                    case "pickup":
                        self.driver.Actuators(2450, 1250, 0)
                        time.sleep(0.3)
                        self.driver.Actuators(2450, 1950, 0)
                        time.sleep(0.1)
                        self.driver.Actuators(2450, 1950, 1)
                        self.driver.Actuators(2450, 1950, 0)
                        time.sleep(0.1)
                        self.driver.Actuators(1500, 1950, 0)
                        time.sleep(1)
                        self.driver.immediateOperation((-30000+4363),150000)
                        self.state0 = "detection"

                    case "detection":
                        for j in range(0, 260, 10):
                            self.driver.Actuators(max(1500-j, 1250), 1950, 0)
                            time.sleep(0.1)  
                        self.driver.Actuators(1250, 1950, 0)
                        time.sleep(0.1)
                        self.state0 = "analyze"

                    
                    case "analyze":
                        self.driver.Actuators(1250, 1950, 0)
                        for k in range (0,5):
                                found, lowerright = self.camera.getDelta()
                                print("IR", found, lowerright)
                                time.sleep(0.1)
                        time.sleep(0.1)
                        self.state0 = "cut"
                        
                    case "cut":
                        for i in range(0, 310, 10):
                            self.driver.Actuators(max(1250-i, 1100), 1950, 0)
                            # time.sleep(0.035)                        
                            time.sleep(0.1)
                        time.sleep(0.1)  
                        self.state0 = "drop off"

                    case "drop off":
                        self.driver.Actuators(2450, 1950, 0)
                        time.sleep(1)
                        self.driver.Actuators(2450, 800, 0)
                        time.sleep(0.5)
                        self.driver.Actuators(2450, 800, 0)
                        time.sleep(0.5)
                        self.driver.Actuators(1500, 1250, 0)
                        time.sleep(0.5)
                        self.state0 = "return"

                    case "return":
                        
                        self.driver.immediateOperation((30000-4363), 150000)
                        
                        self.state0 = "loop"
                    
                    case _:
                        ...


               
               
            else:
                time.sleep(0.03)            

            
            time.sleep(0.08)

class camera():
    def __init__(self):
        self.process_list = []         
        self.QueueShutdown = Queue()
        self.QueueCurrentFrame = Queue()
        self.QueueDetection = Queue()
        self.QueueDetectionMode = Queue()
        self.QueueCorrectionFromCV = Queue()
        self.xD:float = 100
        self.detection = []
        self.convertedImage = None
        
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"format":'XRGB8888', "size":(720,480)}, controls={'FrameRate':120}))
        self.picam2.start()
        self.scale_factor = 3

        #filename = './img/f2.jpg' #9
        template_filename = 'ear_k.jpg'

        #frame = cv2.imread(filename)
    
        # Get the width and height of the frame
        # Get a new frame
        self.frame = self.picam2.capture_array()        
        self.height, self.width = self.frame.shape[:2]
    
        # Define ROI: (x1, y1, x2, y2)
        self.ear_roi = (int(0.2 * self.width), int(0.2 * self.height), int(0.75 * self.width), int(0.8 * self.height))

        # Create an instance of the shrimp
        self.shrimp = find_shrimp_features()

        self.shrimp.generate_scaled_and_rotated_templates(
       template_filename, 
        low_scale_factor=0.75, 
        high_scale_factor=1.25, 
        scale_step=9,
        scale_divider=self.scale_factor,
        rotation_range=2,
        rotation_steps=10

    

        )
        
    def getDelta(self)->[bool,int]:
        # Get a new frame
        self.frame = self.picam2.capture_array()        
    
        # record start time
        time_start = time.perf_counter()

        # Find ear
        found, topleft, lowerright = self.shrimp.find_ear_mt(self.frame, self.ear_roi, match_threshold=0.45, scale_divider=self.scale_factor)
    
        if found:
            #print(f"Ear found! Top-left corner: {topleft}, Bottom-right corner: {lowerright}")
            cv2.rectangle(self.frame, topleft, lowerright, (0, 255, 0), 1)
            # Calculate the average intensity
            
        else:
            print("No match found")

        # record end time
        time_end = time.perf_counter()
        # calculate the duration
        time_duration = time_end - time_start
        # report the duration
        print(f'Took {time_duration} seconds')

        # Draw ROI rectangle on the frame
        cv2.rectangle(self.frame, (self.ear_roi[0], self.ear_roi[1]), (self.ear_roi[2], self.ear_roi[3]), (255, 0, 0), 2)
        opencv_image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)               
        self.convertedImage = Image.fromarray(opencv_image)
        
        return found, lowerright
                
   
    def getCurrentFrame(self):
        # print("get current image", self.convertedImage)
        return self.convertedImage


class governor():
    def __init__(self):   
        self.driver = driver()
        self.camera = camera()
        #self.camera = None
        self.statemachine = statemachine(self.driver, self.camera)
        self.shutdown:bool = False
        self.state_0:str = ""
        self.state_1:str = ""
        self.process_list=[]
        self.startup()
        self.currentFrame=None
        self.error:str=""

   
    def collectUpdates(self)->None:
        while True:
            if self.shutdown:
                return
            self.state_0, self.state_1, self.error = self.statemachine.getStates()
            
            self.currentFrame = self.camera.getCurrentFrame()
            time.sleep(0.05)
            
    
    def getCurrentState(self):
        return [self.state_0, self.state_1, self.error]
    
    def getCurrentFrame(self):
        return self.currentFrame
        
    def startup(self)->None:
        process = Thread(target=self.collectUpdates)
        self.process_list.append(process)  
        
        for process in self.process_list:
            process.start()

   
    def userInput(self, userInputString:str)->None:
        # print("user input", userInputString)
        match userInputString:
            case "start":
                self.statemachine.start()
            case "hold":
                self.statemachine.stop(True)
            case "resume":
                self.statemachine.stop(False)
            case "cycle":
                self.statemachine.setCycling(True)
            case "single":
                self.statemachine.setCycling(False)
            case "reset":
                self.statemachine.reset()
            case "homing":
                self.statemachine.homing()
                self.camera.getDelta()
            case _:
                ...

def main()->None:   
    ctrl = governor()
    # ctrl.start_controller()

if __name__ == "__main__":
    main()
