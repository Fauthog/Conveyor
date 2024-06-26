from threading import Thread
import time
from multiprocessing import Process, Queue
# import cv2
from PIL import Image
# from picamera2 import Picamera2
from mv_shrimp import find_shrimp_features
import serial
from alpha5 import Alpha5Client
 

class driver():

    def __init__(self, ): 
        self.arduinoPort = "COM4"      
        self.setupSerialToArduino()
        self.setupAlpha()

    def setupAlpha(self)->None:
        self.alpha5 = Alpha5Client("COM12")
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
        print('Wait for the operatio to complete')
        self.alpha5.wait_operation_complete(id=1)
        print('done!')        
     
        print('Set [S-ON] to OFF')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='OFF') # [S-ON]->CONT9

    def immediateOperation(self, Units:int, Speed:int)->None:
        print('Set [S-ON] to ON')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='ON') # [S-ON]->CONT9

        print('Send immediate operation setting')
        self.alpha5.send_immediate_operation_setting(id=1, units=Units, speed=Speed)
        print('Set [START] to ON')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=1, state='ON') # [START]->CONT10
        print('Set [START] to OFF')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=1, state='OFF') # [START]->CONT10


        print('Wait for the operatio to complete')
        self.alpha5.wait_operation_complete(id=1)


        print('Set [S-ON] to OFF')
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
        self.driver.Actuators(970, 1250, 0)
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

    def stop(self, hold:bool):
        self.hold=hold
        self.driver.hold(hold)

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
                        self.driver.Actuators(970, 1250, 0)
                        time.sleep(1)
                        self.driver.immediateOperation(30000, 100000)
                        self.state0="loop"
                        

                    case "loop":                       
                            if self.cycling:
                                self.Start = True
                            else:
                                self.Start = False
                            self.state0 = "wait"
                    
                    case "wait":
                        self.driver.Actuators(2000, 1250, 0)
                        if self.Start:
                            self.state0 = "start"                        

                    case "start":
                        # time.sleep(2)
                        self.driver.Actuators(2000, 1250, 0)
                        time.sleep(0.1)
                        self.state0 = "synchronize"
                        

                    case "synchronize":
                        self.driver.Actuators(2000, 1250, 0)
                        time.sleep(0.1)                    
                        self.state0 = "pickup"
                    
                    case "pickup":
                        self.driver.Actuators(2450, 1250, 0)
                        time.sleep(0.4)
                        self.driver.Actuators(2450, 1850, 0)
                        time.sleep(0.1)
                        self.driver.Actuators(2450, 1850, 1)
                        self.driver.Actuators(2450, 1850, 0)
                        time.sleep(0.1)
                        self.driver.Actuators(1170, 1850, 0)
                        time.sleep(0.5)
                        self.driver.immediateOperation(-30000, 100000)
                        self.state0 = "detection"

                    case "detection":
                        self.state0 = "analyze"

                    
                    case "analyze":
                        self.driver.Actuators(1170, 1850, 0)
                        time.sleep(0.04)
                        self.state0 = "cut"
                        
                    case "cut":
                        for i in range(5, 210, 10):
                            self.driver.Actuators(1170-i, 1850, 0)
                            time.sleep(0.05)                        
                        
                        self.state0 = "drop off"

                    case "drop off":
                        self.driver.Actuators(2450, 1850, 0)
                        time.sleep(1)
                        self.driver.Actuators(2450, 800, 0)
                        time.sleep(0.5)
                        self.driver.Actuators(2000, 1250, 0)
                        self.state0 = "return"

                    case "return":
                        
                        self.driver.immediateOperation(30000, 100000)
                        
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
        

        process = Process(target=self.camera, args=(self.QueueShutdown, self.QueueCurrentFrame, self.QueueCorrectionFromCV, self.QueueDetection, self.QueueDetectionMode,))      
        self.process_list.append(process)     
       
        process = Thread(target=self.convertCapturedImage, args=(self.QueueShutdown, self.QueueCurrentFrame,))
        self.process_list.append(process) 

        process = Thread(target=self.DetectionFromCV, args=(self.QueueShutdown, self.QueueDetection,))
        self.process_list.append(process) 

        process = Thread(target=self.CorrectionFromCV, args=(self.QueueShutdown, self.QueueCorrectionFromCV,))
        self.process_list.append(process) 

        for process in self.process_list:
            process.start()
    
    def setDetectionMode(self, detectionMode:str)->None:
        self.QueueDetectionMode.put(detectionMode)

    def shutdown(self)->None:
        self.QueueShutdown.put(True)

    def getCorrectionFromCV(self)->float:
        return self.xD
    
    def getDetectionFromCV(self):
        return self.detection

    def CorrectionFromCV(self, QueueShutdown, QueueCorrectionFromCV)->None:
        # print("convert image")
        shutdown:bool = False
        xD :float = 0.0
        while True:
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown convert image")              
                return
            if not QueueCorrectionFromCV.empty():
                xD = QueueCorrectionFromCV.get()
                while not QueueCorrectionFromCV.empty():
                    xD = QueueCorrectionFromCV.get()
                self.xD = xD
                # print("self.xD", self.xD)
            else:
                time.sleep(0.01)

    
    def DetectionFromCV(self, QueueShutdown, QueueDetection)->None:
        shutdown:bool = False
        detection :float = 0.0
        while True:
            if not QueueShutdown.empty():
                shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown convert image")              
                return
            if not QueueDetection.empty():
                detection = QueueDetection.get()
                while not QueueDetection.empty():
                    detection = QueueDetection.get()
                self.detection = detection
            else:
                time.sleep(0.01)


    def camera(self, QueueShutdown, QueueCurrentFrame, QueueCorrectionFromCV, QueueDetection, QueueDetectionMode)->None:
        #setup camera
        shutdown:bool = False
        detectionError:str=""
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
        shrimp.generate_scaled_and_rotated_templates(
        '/home/admin/Conveyor/template1s.jpg', 
        low_scale_factor=0.75, 
        high_scale_factor=1.25, 
        scale_step=10,
        scale_divider=3,
        rotation_range=10,
        rotation_steps=2
    )
        
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

                    
                    QueueDetection.put([False, False, "Off"])

                case "Nose":  
                    nose_roi = (0, int(0.3 * height), int(0.5 * width), int(0.7 * height)) 
                    found_nose, topleft, lowerright = shrimp.find_nose(frame, nose_roi, nose_dark_threshold, nose_area_threshold)

                    QueueDetection.put([found_nose, False, "Nose"])
                    # print("Nose", found_nose)
                    if found_nose:   
                        cv2.rectangle(frame, topleft, lowerright, (0, 0, 255), 1)                          
                    # ear = False
                    # positive = 0
                    # negative = 0
                    # first_detection = time.perf_counter()
                case "Ear":
                    ear_roi = (int(0.2 * width), int(0.3 * height), int(0.85 * width), int(0.7 * height))
                    found_ear, topleft, lowerright = shrimp.find_ear_mt(frame, ear_roi, match_threshold=0.70, scale_divider=3)
                    # if found_ear and ear==False:
                    #     ear = True
                        # first_detection = time.perf_counter()
                    # if ear:
                    # if found_ear:
                    #     positive +=1
                    # else:
                    #     negative +=1

                    QueueDetection.put([False, found_ear, "Ear"])
                    # print("ear", found_ear)
                    if found_ear:    
                        cv2.rectangle(frame, topleft, lowerright, (0, 255, 0), 1)
                        # blade_mm=(500-intercept)/slope
                        # distance_mm=(lowerright[0]-intercept)/slope
                        # difference = blade_mm - distance_mm - 5.145741756037534
                        difference = (500-lowerright[0])*0.1
                        # print("difference", difference)
                        # if abs(difference) < 0.1:
                        #     delta = time.perf_counter() - first_detection 
                        #     dps = (positive + negative)/delta
                        #     print("performance:", "T:", positive, "F:", negative, "DPS", dps )
                        QueueCorrectionFromCV.put(difference)
                
                case _:
                    QueueDetection.put([False, False, ""])
                     
           
            cv2.line(frame, [500,0], [500,480], (255, 0, 0), 2)
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
                # cv2.imwrite("test.jpg", opencv_image)
                self.convertedImage = Image.fromarray(opencv_image)
            else:
                time.sleep(0.005)  
   
    def getCurrentFrame(self):
        # print("get current image", self.convertedImage)
        return self.convertedImage


class governor():
    def __init__(self):   
        self.driver = driver()
        # self.camera = camera()
        self.camera = None
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
            
            # self.currentFrame = self.camera.getCurrentFrame()
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
            case _:
                ...

def main()->None:   
    ctrl = governor()
    # ctrl.start_controller()

if __name__ == "__main__":
    main()
