# import RPi.GPIO as GPIO
import time
import serial



class demo():
    def __init__(self):
        self.arduinoPort = "COM4"
        self.alpha5SmartPort = ""
        self.bigServo:int = 970
        self.smallServo:int = 1250
        self.solenoid:int =0
        self.setupSerialToArduino()
        


    def setupSerialToArduino(self)->None:
        self.arduino = serial.Serial(port=self.arduinoPort, baudrate=115200, timeout=2)
        # print(self.arduino.readline())
        # print(self.arduino.readline())
    
    def writeToArduino(self)->None:
          
                # command = str(int(self.bigServo)) + ";" + str(int(self.smallServo))
                
            if not self.arduino.is_open:
                print("not open")
                self.arduino.open()
            cmd = str(self.bigServo) + "," + str(self.smallServo) + "," + str(self.solenoid)
            print("cmd:", cmd)
            self.arduino.write((cmd + "\r\n").encode('utf-8'))
            # print(self.arduino.readline())
            # print(self.arduino.readline())
            # print()
            
            

    def cleanUpArduinoConnection(self)->None:
         if self.arduino !="None":
            # sendStr = "<DSA,0,0.0>"
            # self.current_port.write(sendStr.encode())
            # self.current_port.write(sendStr.encode())
            self.arduino.flushInput()
            self.arduino.flushOutput()
            self.arduino.close()

    def getUserInput(self)->bool:
        user_input = input("proceed?:").lower()
        if len(user_input)>0:
            if user_input[0] == "e":
                self.cleanUpArduinoConnection()
                return False
        return True
    
    
    def loop(self)->None:
        print("initial")
        if self.getUserInput(): 
                print("initial")   
                self.smallServo=1250
                self.bigServo=2000
                self.solenoid=0
                self.writeToArduino()
        else:
            return

        while True:        
            
            if self.getUserInput(): 
                    print("prepare to grab")   
                    self.smallServo=1250
                    self.bigServo=2450                   
                    self.writeToArduino()
            else:
                return
            
            if self.getUserInput(): 
                    print("grab shrimp")   
                    self.smallServo=2000
                    self.bigServo=2450 
                    self.solenoid=1
                    self.writeToArduino()
                    self.solenoid=0                  
                    self.writeToArduino()
            else:
                return
            
            if self.getUserInput():  
                    print("move up to cam")  
                    self.smallServo=1850
                    self.bigServo=1170                 
                    self.writeToArduino()
            else:
                return

            if self.getUserInput():  
                    print("cut")  
                    self.smallServo=1850
                    for i in range(5, 210, 10):
                        self.bigServo=1170-i
                        self.writeToArduino()
            else:
                return
            
            if self.getUserInput():  
                    print("prepare for drop off")  
                    self.smallServo=1850
                    self.bigServo=2000                 
                    self.writeToArduino()
            else:
                return
            
            if self.getUserInput(): 
                    print("drop off")                     
                    self.bigServo=2450   
                    self.writeToArduino()
                    time.sleep(0.1)
                    self.smallServo=800               
                    self.writeToArduino()
            else:
                return
            
            if self.getUserInput():  
                    print("start")  
                    self.smallServo=1250
                    self.bigServo=2000                  
                    self.writeToArduino()
            else:
                return



def main()->None:
    d=demo()
    d.loop()

if __name__ == "__main__":
    main()