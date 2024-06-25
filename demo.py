import RPi.GPIO as GPIO
import time
import serial


class demo():
    def __init__(self):
        self.arduinoPort = "/dev/ttyUSB0"
        self.alpha5SmartPort = ""

    def setupSerialToArduino(self)->None:
        self.arduino = serial.Serial(port=self.arduinoPort, baudrate=9600, timeout=2)
        ...
    
    def writeToArduino(self, command:str)->None:
        ...

    def cleanUpArduinoConnection(self)->None:
        ...

    def setupSerialToAlpha5Smart(self)->None:
        ...
    def cleanUpAlpha5SmartConnection(self)->None:
        ...
    
    def loop()->None:
        while True:
            user_input = input("test:").lower()
            if user_input[0] == "e":
                return


def main()->None:
    demo=demo()

if __name__ == "__main__":
    main()