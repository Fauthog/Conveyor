from threading import Thread
import RPi.GPIO as GPIO
import time
from multiprocessing import Process, Queue
import cv2
from PIL import Image
from picamera2 import Picamera2
from mv_shrimp import find_shrimp_features
 

class driver():
    def __init__(self, ): 
        self.process_list = []
        self.QueueShutdown = Queue()
        
        self.QueueSolenoidState = Queue()
        self.speedFactor:float = 1
        
        # driver0
        self.QueueClosedLoop0 = Queue()
        self.QueueCurrentPosition0 = Queue()
        self.QueueXD0 = Queue()
        self.QueueInitPosition0 = Queue()        
        self.currentPositionInPulse0:int = 0
        self.currentPositionInMillimeter0:float = 0.0
        self.servo1state_0:str = ""
        self.servo2state_0:str = ""
        self.QueueHold0 = Queue()
        self.QueueLL0 = Queue()
        # driver1
        self.QueueClosedLoop1 = Queue()
        self.QueueCurrentPosition1 = Queue()
        self.QueueXD1 = Queue()
        self.QueueInitPosition1 = Queue()        
        self.currentPositionInPulse1:int = 0
        self.currentPositionInMillimeter1:float = 0.0
        self.servo1state_1:str = ""
        self.servo2state_1:str = ""
        self.QueueHold1 = Queue()
        self.QueueLL1 = Queue()

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

        self.setupGPIO()

        process = Process(target=self.driver0, args=(self.QueueShutdown, self.QueueHold0, self.QueueXD0, self.QueueCurrentPosition0, self.QueueInitPosition0,))      
        self.process_list.append(process)    

        # process = Process(target=self.driver1, args=(self.QueueShutdown, self.QueueHold1, self.QueueXD1, self.QueueCurrentPosition1, self.QueueInitPosition1,))      
        # self.process_list.append(process)  

        process = Process(target=self.solenoidstate, args=(self.QueueShutdown, self.QueueSolenoidState,))      
        self.process_list.append(process)  

        process = Thread(target=self.currentPositionConverter, args=(self.QueueShutdown, self.QueueCurrentPosition0, self.QueueCurrentPosition1,))
        self.process_list.append(process)

        for process in self.process_list:
            process.start()

    def shutdown(self):
        print("shuttign down driver")
        try:
            for process in self.process_list:
                i=0
                while process.is_alive():
                    i += 1
                    self.QueueShutdown.put(True)
                    if i > 10:
                        process.close()
                    if i > 15:
                        process.terminate()
                    
            if not self.QueueCurrentPosition.empty():
                while not self.QueueShutdown.empty():
                    try:
                        self.QueueShutdown.get()
                    except:
                        self.QueueShutdown.close()
            if not self.QueueClosedLoop.empty():
                while not self.QueueShutdown.empty():
                    try:
                        self.QueueShutdown.get()
                    except:
                        self.QueueShutdown.close()
            if not self.QueueShutdown.empty():
                while not self.QueueShutdown.empty():
                    try:
                        self.QueueShutdown.get()
                    except:
                        self.QueueShutdown.close()
            if not self.QueueInitPosition.empty():
                while not self.QueueInitPosition.empty():
                    try:
                        self.QueueInitPosition.get()
                    except:
                        self.QueueInitPosition.close()
        except:
            print("failure on driver shutdown")


        print("driver is shut down")

    def currentPositionConverter(self,QueueShutdown, QueueCurrentPosition0, QueueCurrentPosition1)->None:
        print("convert position")
        while True:
            if not QueueShutdown.empty():
                if QueueShutdown.get():
                    print("shutdown convert position")              
                return              
                
            if not QueueCurrentPosition0.empty():
                position0 = QueueCurrentPosition0.get()
                while not QueueCurrentPosition0.empty():
                    try:
                        position0 = QueueCurrentPosition0.get()
                    except:
                        ...
                self.currentPositionInMillimeter0 = position0 * self.mm_per_step

            if not QueueCurrentPosition1.empty():
                position1 = QueueCurrentPosition1.get()
                while not QueueCurrentPosition1.empty():
                    try:
                        position1 = QueueCurrentPosition1.get()
                    except:
                        ...
                self.currentPositionInMillimeter1 = position1 * self.mm_per_step
              
            else:
                time.sleep(0.001)

    def getCurrentPosition0(self)->float:        
        return self.currentPositionInMillimeter0 
    
    def getCurrentPosition1(self)->float:        
        return self.currentPositionInMillimeter1 
                
    def setupGPIO(self)->None:
        self.STEP_PIN = 18
        self.DIR_PIN = 27
        self.LL_PIN = 6
        self.Solenoid_PIN = 22
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.LL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.Solenoid_PIN, GPIO.OUT)

    def isInitPosition(self)->bool:
        return GPIO.input(self.LL_PIN)==0 and self.currentPositionInPulse0 == 0 and self.currentPositionInPulse1 == 1473
    
    def initPosition0(self)->None:
        while GPIO.input(self.LL_PIN)==1:
                    GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * 60)
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    time.sleep(delay)                                       
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    time.sleep(delay)  
        self.currentPositionInPulse0 = 0
        self.currentPositionInMillimeter0 = 0.0     

    def initPosition1(self)->None:      
        self.currentPositionInPulse1 = 1473
        self.currentPositionInMillimeter1 = 350   

    def LL0(self)->None:
        while GPIO.input(self.LL_PIN)==1:
                    GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * 60)
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    time.sleep(delay)                                       
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    time.sleep(delay)  
        self.currentPositionInPulse0 = 0
        self.currentPositionInMillimeter0 = 0.0

    def LL1(self)->None:
        self.currentPositionInPulse1 = 0   
        self.currentPositionInMillimeter1 = 0
    
    def touchLL0(self)->None:
        self.QueueLL0.put(True)

    def touchLL1(self)->None:
        self.QueueLL1.put(True)

    def hold(self, hold:bool)->None:
        self.QueueHold0.put(hold)
        self.QueueHold1.put(hold)

    def zero0(self)->None:
        self.QueueInitPosition0.put(True)

    def zero1(self)->None:
        self.QueueInitPosition1.put(True)

    def goto0(self, positionInMm:float, speedInRpm:int, speedLimit:int, external:bool)->None:
        setpointPositionSteps = int(positionInMm/self.mm_per_step)      
        setpoint = [setpointPositionSteps, speedInRpm, speedLimit, external]
        self.QueueXD0.put(setpoint)

    def goto1(self, positionInMm:float, speedInRpm:int, speedLimit:int, external:bool)->None:
        setpointPositionSteps = int(positionInMm/self.mm_per_step)      
        setpoint = [setpointPositionSteps, speedInRpm, speedLimit, external]
        self.QueueXD1.put(setpoint)

    def driver0(self, QueueShutdown, QueueHold0, QueueXD0, QueueCurrentPosition0, QueueInitPosition0, QueueLL0)->None:
        print("driver0 process starting")
        
        self.initPosition0()
        currentPosition:int = 0
        currentSpeed: int = 0
        hold:bool = False
        
        externalXD:bool = True
        xD:int = 0
        xDD:int = 0
        positionSetpoint:int = 0
        speedSetpoint:int = 0
        speedLimit:int = 0
        limitedSpeed:float = 0.0
        PxD:float = 1.0
        PxDD:float = 1.0
        IxDD:float = 0.3
        lastIxDD:float = 0.0
        maxSpeedDelta0:int = 25
        maxSpeedDelta:int = 100

        while True:
            start_time = time.perf_counter()
            if not QueueShutdown.empty():
                if QueueShutdown.get():
                    print("shutdown driver0")              
                    return
                
            
            while not QueueInitPosition0.empty(): # empty the Queue
                try:
                    if QueueInitPosition0.get():
                        self.initPosition0()
                        currentPosition = 0
                        currentSpeed = 0
                        lastIxDD = 0
                except:
                    ...
            while not QueueLL0.empty(): # empty the Queue
                try:
                    if QueueLL0.get():
                        self.LL0()
                        currentPosition = 0
                        currentSpeed = 0
                        lastIxDD = 0
                except:
                    ...

            if not QueueHold0.empty():
                hold = QueueHold0.get()
                while not QueueHold0.empty(): # empty the Queue
                    try:
                       hold = QueueHold0.get()
                    except:
                        ...

            if not QueueXD0.empty():
                positionSetpoint, speedSetpoint, speedLimit, externalXD = QueueXD0.get()
                while (not QueueXD0.empty()): # empty the Queue
                    try:
                        positionSetpoint, speedSetpoint, speedLimit = QueueXD0.get()
                    except:
                        ...

            if externalXD:
                xD = positionSetpoint
                if xD < 0:
                    xDD = -speedSetpoint
                else:
                    xDD = speedSetpoint                
            else:
                xD = positionSetpoint - currentPosition
                if xD < 0:
                    xDD = -speedSetpoint - currentSpeed
                else:
                    xDD = speedSetpoint - currentSpeed
               

            if not hold:

                # controller               
                IxDD = IxDD + lastIxDD
                requestedSpeed = xD * PxD + xDD * (PxDD + IxDD)
                lastIxDD = xDD * IxDD

                # rate limiter
                speedDelta = requestedSpeed - currentSpeed

                if currentSpeed == 0:
                    limitedSpeed = min(max(speedDelta, -maxSpeedDelta0), maxSpeedDelta0)
                else:
                    limitedSpeed=currentSpeed + min(max(speedDelta, -maxSpeedDelta), maxSpeedDelta)

                # slope to limits
                ULSpeed = (self.steps - currentPosition) * maxSpeedDelta 
                LLSpeed = -currentPosition * maxSpeedDelta
                limitedSpeed = min(max(limitedSpeed, LLSpeed),ULSpeed)              
                
                # absolute speed limiter
                limitedSpeed = min(max(limitedSpeed, -speedLimit), speedLimit)

                if limitedSpeed < 0: 
                        GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                        delay:float = 0.5 * 60.0 / (self.steps_per_rev * abs(limitedSpeed))
                        call_time_delta = time.perf_counter() - start_time
                        delta_delay = delay - call_time_delta
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                     
                        if 0 <= delta_delay <0.03:
                            time.sleep(delta_delay)
                        else:
                            time.sleep(delay)
                        
                        currentPosition -= 1
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                     
                        time.sleep(delay)
                        GPIO.output(self.STEP_PIN, GPIO.LOW)

                elif limitedSpeed > 0:
                    GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * abs(limitedSpeed))
                    call_time_delta = time.perf_counter() - start_time
                    delta_delay = delay - call_time_delta
                    
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    
                    if 0 <= delta_delay <0.03:
                        time.sleep(delta_delay)
                    else:
                        time.sleep(delay)
                    currentPosition += 1
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    
                    time.sleep(delay)
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                
                currentSpeed = limitedSpeed

            else:
                 # ramp down to stop
                if currentSpeed > 1:  # case forward
                    limitedSpeed = max(currentSpeed-60, 1) 
                    GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * limitedSpeed)
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)                   
                    time.sleep(delay)
                    currentPosition +=1
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    
                    time.sleep(delay)
                    currentSpeed = limitedSpeed

                elif currentSpeed < -1: # case reverse
                    limitedSpeed = min(currentSpeed+60, -1)
                    GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * abs(limitedSpeed))
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)                    
                    time.sleep(delay)
                    currentPosition -= 1
                    GPIO.output(self.STEP_PIN, GPIO.LOW)                    
                    time.sleep(delay)
                    currentSpeed = limitedSpeed
                else:
                    # if we are stopped, we wait
                    time.sleep(0.1)
                    currentSpeed = 0
        
            QueueCurrentPosition0.put(currentPosition) 

    def driver1(self, QueueShutdown, QueueHold1, QueueXD1, QueueCurrentPosition1, QueueInitPosition1, QueueLL1)->None:
        print("driver0 process starting")
        
        self.initPositionUL1()
        currentPosition:int = 0
        currentSpeed: int = 0
        hold:bool = False
        
        externalXD:bool = True
        xD:int = 0
        xDD:int = 0
        positionSetpoint:int = 0
        speedSetpoint:int = 0
        speedLimit:int = 0
        limitedSpeed:float = 0.0
        PxD:float = 1.0
        PxDD:float = 1.0
        IxDD:float = 0.3
        lastIxDD:float = 0.0
        maxSpeedDelta0:int = 25
        maxSpeedDelta:int = 100

        while True:
            start_time = time.perf_counter()
            if not QueueShutdown.empty():
                if QueueShutdown.get():
                    print("shutdown driver0")              
                    return
                
            
            while not QueueInitPosition1.empty(): # empty the Queue
                try:
                    if QueueInitPosition1.get():
                        self.initPosition1()
                        currentPosition = 0
                        currentSpeed = 0
                        lastIxDD = 0
                except:
                    ...

            while not QueueLL1.empty(): # empty the Queue
                try:
                    if QueueLL1.get():
                        self.LL1()
                        currentPosition = 0
                        currentSpeed = 0
                        lastIxDD = 0
                except:
                    ...

            if not QueueHold1.empty():
                hold = QueueHold1.get()
                while not QueueHold1.empty(): # empty the Queue
                    try:
                       hold = QueueHold1.get()
                    except:
                        ...

            if not QueueXD1.empty():
                positionSetpoint, speedSetpoint, speedLimit, externalXD = QueueXD1.get()
                while (not QueueXD1.empty()): # empty the Queue
                    try:
                        positionSetpoint, speedSetpoint, speedLimit = QueueXD1.get()
                    except:
                        ...

            if externalXD:
                xD = positionSetpoint
                if xD < 0:
                    xDD = -speedSetpoint
                else:
                    xDD = speedSetpoint                
            else:
                xD = positionSetpoint - currentPosition
                if xD < 0:
                    xDD = -speedSetpoint - currentSpeed
                else:
                    xDD = speedSetpoint - currentSpeed
               

            if not hold:

                # controller               
                IxDD = IxDD + lastIxDD
                requestedSpeed = xD * PxD + xDD * (PxDD + IxDD)
                lastIxDD = xDD * IxDD

                # rate limiter
                speedDelta = requestedSpeed - currentSpeed

                if currentSpeed == 0:
                    limitedSpeed = min(max(speedDelta, -maxSpeedDelta0), maxSpeedDelta0)
                else:
                    limitedSpeed=currentSpeed + min(max(speedDelta, -maxSpeedDelta), maxSpeedDelta)

                # slope to limits
                ULSpeed = (self.steps - currentPosition) * maxSpeedDelta 
                LLSpeed = -currentPosition * maxSpeedDelta
                limitedSpeed = min(max(limitedSpeed, LLSpeed),ULSpeed)              
                
                # absolute speed limiter
                limitedSpeed = min(max(limitedSpeed, -speedLimit), speedLimit)

                if limitedSpeed < 0: 
                        GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                        delay:float = 0.5 * 60.0 / (self.steps_per_rev * abs(limitedSpeed))
                        call_time_delta = time.perf_counter() - start_time
                        delta_delay = delay - call_time_delta
                        GPIO.output(self.STEP_PIN, GPIO.LOW)
                       
                        if 0 <= delta_delay <0.03:
                            time.sleep(delta_delay)
                        else:
                            time.sleep(delay)
                        
                        currentPosition -= 1
                        GPIO.output(self.STEP_PIN, GPIO.HIGH)
                     
                        time.sleep(delay)
                        GPIO.output(self.STEP_PIN, GPIO.LOW)

                elif limitedSpeed > 0:
                    GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * abs(limitedSpeed))
                    call_time_delta = time.perf_counter() - start_time
                    delta_delay = delay - call_time_delta
                    
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    
                    if 0 <= delta_delay <0.03:
                        time.sleep(delta_delay)
                    else:
                        time.sleep(delay)
                    currentPosition += 1
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    
                    time.sleep(delay)
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                
                currentSpeed = limitedSpeed

            else:
                 # ramp down to stop
                if currentSpeed > 1:  # case forward
                    limitedSpeed = max(currentSpeed-60, 1) 
                    GPIO.output(self.DIR_PIN, GPIO.LOW) # forward
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * limitedSpeed)
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)                   
                    time.sleep(delay)
                    currentPosition +=1
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    
                    time.sleep(delay)
                    currentSpeed = limitedSpeed

                elif currentSpeed < -1: # case reverse
                    limitedSpeed = min(currentSpeed+60, -1)
                    GPIO.output(self.DIR_PIN, GPIO.HIGH) # reverse
                    delay:float = 0.5 * 60.0 / (self.steps_per_rev * abs(limitedSpeed))
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)                    
                    time.sleep(delay)
                    currentPosition -= 1
                    GPIO.output(self.STEP_PIN, GPIO.LOW)                    
                    time.sleep(delay)
                    currentSpeed = limitedSpeed
                else:
                    # if we are stopped, we wait
                    time.sleep(0.1)
                    currentSpeed = 0
        
            QueueCurrentPosition1.put(currentPosition) 

    def servo1state0(self, state:str)->None:
        self.servo1state_0 = state[:]
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
                       
    def servo2state0(self, state:str)->None:
        self.servo2state_0 = state[:]
        match state:
            case "open":
                GPIO.output(self.SERVO2_PIN, GPIO.HIGH)
            case "close":   
                GPIO.output(self.SERVO2_PIN, GPIO.LOW)
            case _:
                GPIO.output(self.SERVO2_PIN, GPIO.LOW)
                   
    def servo1state1(self, state:str)->None:
        self.servo1state_1 = state[:]
        match state:
            case "0":
                ...
                # GPIO.output(self.SERVO1_PIN1, GPIO.LOW)
                # GPIO.output(self.SERVO1_PIN2, GPIO.LOW)
            case "1":
                ...
                # GPIO.output(self.SERVO1_PIN1, GPIO.LOW)
                # GPIO.output(self.SERVO1_PIN2, GPIO.HIGH)
            case "2":
                ...
                # GPIO.output(self.SERVO1_PIN1, GPIO.HIGH)
                # GPIO.output(self.SERVO1_PIN2, GPIO.LOW)
            case "3":
                ...
                # GPIO.output(self.SERVO1_PIN1, GPIO.HIGH)
                # GPIO.output(self.SERVO1_PIN2, GPIO.HIGH)
            case _:
                ...
                # GPIO.output(self.SERVO1_PIN1, GPIO.LOW)
                # GPIO.output(self.SERVO1_PIN2, GPIO.LOW)
                       
    def servo2state1(self, state:str)->None:
        self.servo2state_1 = state[:]
        match state:
            case "open":
                ...
                # GPIO.output(self.SERVO2_PIN, GPIO.HIGH)
            case "close":  
                ... 
                # GPIO.output(self.SERVO2_PIN, GPIO.LOW)
            case _:
                ...
                # GPIO.output(self.SERVO2_PIN, GPIO.LOW)
                

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

    def pulseSolenoid(self)->None:
            self.QueueSolenoidState.put(True)
        
    def solenoidstate(self, QueueShutdown, QueueSolenoidState)->None:        
        print("start solenoidstate")    
        shutdown = False
        while True:
            if not QueueShutdown.empty():
                    shutdown = QueueShutdown.get()
            if shutdown:  
                print("shutdown solenoid")              
                return
            if not QueueSolenoidState.empty():
                if QueueSolenoidState.get():                
                    GPIO.output(self.Solenoid_PIN, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(self.Solenoid_PIN, GPIO.LOW)
            else:
                time.sleep(0.01)

class statemachine():
    def __init__(self, driver, camera): 
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
        self.hold:bool = True

        self.QueueCorrectionFromCV = self.camera.QueueCorrectionFromCV

        process = Thread(target=self.statemachine)
        self.process_list.append(process)

        for process in self.process_list:
            process.start()
    
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
        return [self.state0, self.state1] 

    def setCycling(self, cycling:bool)->None:
        self.cycling = cycling

    def stop(self, hold:bool):
        self.hold=hold
        self.driver.hold()

    def start(self):
        self.Start = True

    def reset(self)->None:
        self.state0 = "init"
        self.state1 = "init"
        self.error_state = ""
 
    def statemachine(self):
       

        while True:
            if self.shutDown:
                print("shutting down statemachine")
                return
            self.printState()
            if not self.hold:
                match self.state0:                
                    case "init":
                        self.error_state = ""
                        self.driver.zero0()
                        self.driver.servo1state0("0")
                        self.driver.servo2state0("open")
                        self.camera.setDetectionMode("Off")
                        if self.driver.isInitPosition():                       
                            self.state0 = "wait"

                    case "limit":
                        self.driver.touchLL0()
                        self.sleep(0.1)
                        if self.driver.getCurrentPosition0() == 0:
                            if self.cycling:
                                self.Start = True
                            else:
                                self.Start = False
                            self.state0 = "wait"
                    
                    case "wait":
                        if self.Start:
                            self.state0 = "start"
                        else:
                            time.sleep(0.1)

                    case "start":
                        self.driver.servo1state0("0")
                        self.driver.servo2state0("open")
                        self.camera.setDetectionMode("Off")
                        if self.Start and self.state1 == "cut":
                            self.state0 = "synchronize"
                        else:
                            time.sleep(0.1)

                    case "synchronize":
                        self.driver.servo1state0("0")
                        self.driver.servo2state0("close")
                        self.camera.setDetectionMode("Off")
                        self.driver.goto0(0,400, 410, False)
                        if self.driver.getCurrentPosition0() > 30:
                            self.driver.pulseSolenoid()                    
                            self.state0 = "pickup"
                    
                    case "pickup":
                        self.driver.servo1state0("1")
                        self.driver.servo2state0("close")
                        self.camera.setDetectionMode("Off")
                        self.driver.goto0(300, 400, 400, False)
                        if self.driver.getCurrentPosition0() > 150:
                            self.state0 = "detection"

                    case "detection":
                        self.driver.servo1state0("1")
                        self.driver.servo2state0("close")
                        self.camera.setDetectionMode("Ear")
                        self.driver.goto0(301, 400, 400, False)
                        if self.camera.getDetectionFromCV()[0]:
                            self.state0 = "analyze"
                        if self.driver.getCurrentPosition0() > 300:
                            self.state0 = "IR fail"
                        if self.camera.getDetectionFromCV()[1]:
                            self.state0 = "IR fail"
                    
                    case "analyze":
                        self.driver.servo1state0("1")
                        self.driver.servo2state0("close")
                        self.camera.setDetectionMode("Nose")
                        if self.getDetectionfromCV()[0]:
                            xD = self.camera.getCorrectionFromCV()
                            self.driver.goto0(xD, 0, 100, True)
                            print("xD", xD)
                            if xD<0.5:
                                self.state0 = "cut"

                    case "IR fail":
                        self.driver.hold()
                        print("IRFail", self.camera.getDetectionFromCV()[2])
                        self.error_state = self.camera.getDetectionFromCV()[2]

                    case "cut":
                        self.driver.servo1state0("2")
                        self.driver.servo2state0("close")
                        self.camera.setDetectionMode("Off")
                        time.sleep(1)
                        self.state0 = "drop off"

                    case "drop off":
                        self.driver.servo1state0("0")
                        time.sleep(1)
                        self.driver.servo2state0("open")
                        self.cameras.setDetectionMode("Off")
                        time.sleep(1)
                        if self.state1 == "cut":
                            self.state0 = "return"

                    case "return":
                        self.driver.servo1state0("0")
                        self.driver.servo2state0("open")                   
                        self.cameras.setDetectionMode("Off")
                        self.driver.goto0(0, 600, 600, False)
                        if self.driver.getCurrentPosition0() < 1:
                            self.state0 = "limit"
                    
                    case _:
                        ...

                
                
                
                match self.state1:                
                    case "init":
                        self.error_state = ""
                        self.driver.zero1()
                        self.driver.servo1state1("0")
                        self.driver.servo2state1("open")
                        # self.camera.setDetectionMode("Off")
                        if self.driver.isInitPosition():                       
                            self.state1 = "wait"

                    case "limit":
                        self.driver.touchLL1()
                        self.sleep(0.1)
                        if self.driver.getCurrentPosition1() == 0:
                            if self.cycling:
                                self.Start = True
                            else:
                                self.Start = False
                            self.state1 = "start"
                    
                    case "wait":
                        if self.Start:
                            self.state1 = "return"
                        else:
                            time.sleep(0.1)

                    case "start":
                        self.driver.servo1state1("0")
                        self.driver.servo2state1("open")
                        # self.camera.setDetectionMode("Off")
                        if self.Start and self.state0 == "cut":
                            self.state1 = "synchronize"
                        else:
                            time.sleep(0.1)

                    case "synchronize":
                        self.driver.servo1state1("0")
                        self.driver.servo2state1("close")
                        # self.camera.setDetectionMode("Off")
                        self.driver.goto1(0,400, 410, False)
                        if self.driver.getCurrentPosition1() > 30:
                            self.driver.pulseSolenoid()                    
                            self.state1 = "pickup"
                    
                    case "pickup":
                        self.driver.servo1state1("1")
                        self.driver.servo2state1("close")
                        # self.camera.setDetectionMode("Off")
                        self.driver.goto1(300, 400, 400, False)
                        if self.driver.getCurrentPosition1() > 150:
                            self.state1 = "detection"

                    case "detection":
                        self.driver.servo1state1("1")
                        self.driver.servo2state1("close")
                        # self.camera.setDetectionMode("Ear")
                        self.driver.goto1(301, 400, 400, False)
                        # if self.camera.getDetectionFromCV()[0]:
                        #     self.state1 = "analyze"
                        # if self.driver.getCurrentPosition0() > 300:
                        #     self.state1 = "IR fail"
                        # if self.camera.getDetectionFromCV()[1]:
                        #     self.state1 = "IR fail"
                        self.state1 = "analyze"
                    
                    case "analyze":
                        self.driver.servo1state1("1")
                        self.driver.servo2state1("close")
                        # self.camera.setDetectionMode("Nose")
                        # if self.getDetectionfromCV()[0]:
                        #     xD = self.camera.getCorrectionFromCV()
                        #     self.driver.goto1(xD, 0, 100, True)
                        #     print("xD", xD)
                        #     if xD<0.5:
                        #         self.state1 = "cut"
                        self.state1 = "cut"

                    case "IR fail":
                        self.driver.hold()
                        print("IRFail", self.camera.getDetectionFromCV()[2])
                        self.error_state = self.camera.getDetectionFromCV()[2]

                    case "cut":
                        self.driver.servo1state1("2")
                        self.driver.servo2state1("close")
                        # self.camera.setDetectionMode("Off")
                        time.sleep(1)
                        self.state1 = "drop off"

                    case "drop off":
                        self.driver.servo1state1("0")
                        time.sleep(1)
                        self.driver.servo2state1("open")
                        self.cameras.setDetectionMode("Off")
                        time.sleep(1)
                        if self.state0 == "cut":
                            self.state1 = "return"

                    case "return":
                        self.driver.servo1state1("0")
                        self.driver.servo2state1("open")                   
                        # self.cameras.setDetectionMode("Off")
                        self.driver.goto1(0, 600, 600, False)
                        if self.driver.getCurrentPosition1() < 1:
                            self.state1 = "limit"
                    
                    case _:
                        ...
            else:
                time.sleep(0.1)            

            
            time.sleep(0.008)
    

class camera():
    def __init__(self):
        self.process_list = []         
        self.QueueShutdown = Queue()
        self.QueueCurrentFrame = Queue()
        self.QueueDetection = Queue()
        self.QueueDetectionMode = Queue()
        self.QueueCorrectionFromCV = Queue()
        self.xD:float = 0.0
        self.detection = []
        

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
                xd = float(QueueCorrectionFromCV.get())
                while not QueueCorrectionFromCV.empty():
                    xd = float(QueueCorrectionFromCV.get())
                self.xD = xD
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

                    
                    QueueDetection.put([False, False, "Off"])

                case "Nose":  
                    nose_roi = (0, int(0.3 * height), int(0.5 * width), int(0.7 * height)) 
                    found_nose, topleft, lowerright = shrimp.find_nose(frame, nose_roi, nose_dark_threshold, nose_area_threshold)

                    QueueDetection.put([found_nose, False, "Nose"])

                    if found_nose:   
                        cv2.rectangle(frame, topleft, lowerright, (0, 0, 255), 1)                          

                case "Ear":
                    ear_roi = (0, int(0.3 * height), int(0.7 * width), int(0.7 * height))
                    found_ear, topleft, lowerright = shrimp.find_ear(frame, ear_roi, match_threshold=0.8)  

                    QueueDetection.put([found_ear, False, "Ear"])

                    if found_ear:    
                        cv2.rectangle(frame, topleft, lowerright, (0, 255, 0), 1)
                        blade_mm=(480-intercept)/slope
                        distance_mm=(lowerright[0]-intercept)/slope
                        difference = blade_mm - distance_mm - 5.145741756037534
                       
                        QueueCorrectionFromCV.put(difference)
                
                case _:
                    QueueDetection.put([False, False, ""])
                     
           
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
   
    def getCurrentFrame(self):
        # print("get current image", self.convertedImage)
        return self.convertedImage


class governor():
    def __init__(self):   
        self.driver = driver()
        self.camera = camera()
        self.statemachine = statemachine(self.driver, self.camera)
        self.shutdown:bool = False
        self.state_0:str = ""
        self.state_1:str = ""
        self.startup()

   
    def collectUpdates(self)->None:
        while True:
            if self.shutdown:
                return
            self.state_0, self.state_1 = self.statemachine.getStates()
            self.camera.getCurrentFrame()
            time.sleep(0.05)
            
    
    def getCurrentState(self):
        return [self.state_0, self.state_1]
    
    def getCurrentFrame(self):
        return self.currentFrame
        
    def startup(self)->None:
        process = Thread(target=self.collectUpdates)
        self.process_list.append(process)  
        
        for process in self.process_list:
            process.start()

   
    def userInput(self, userInputString:str)->None:
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
            case _:
                ...

def main()->None:   
    ctrl = governor()
    # ctrl.start_controller()

if __name__ == "__main__":
    main()
