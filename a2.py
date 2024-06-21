import RPi.GPIO as GPIO
import time


Solenoid_PIN = 22
SERVO1_PIN1 = 17
SERVO1_PIN2 = 4
SERVO2_PIN = 14
SERVO3_PIN = 14
GPIO.setmode(GPIO.BCM)
GPIO.setup(Solenoid_PIN, GPIO.OUT)
GPIO.setup(SERVO1_PIN1, GPIO.OUT)
GPIO.setup(SERVO1_PIN2, GPIO.OUT)
GPIO.setup(SERVO2_PIN, GPIO.OUT)

def servo1state(state:str)->None:
    match state:
        case "0":
            GPIO.output(SERVO1_PIN1, GPIO.LOW)
            GPIO.output(SERVO1_PIN2, GPIO.LOW)
        case "1":
            GPIO.output(SERVO1_PIN1, GPIO.LOW)
            GPIO.output(SERVO1_PIN2, GPIO.HIGH)
        case "2":
            GPIO.output(SERVO1_PIN1, GPIO.HIGH)
            GPIO.output(SERVO1_PIN2, GPIO.LOW)
        case "3":
            GPIO.output(SERVO1_PIN1, GPIO.HIGH)
            GPIO.output(SERVO1_PIN2, GPIO.HIGH)
        case _:
            GPIO.output(SERVO1_PIN1, GPIO.LOW)
            GPIO.output(SERVO1_PIN2, GPIO.LOW)
                       
def servo2state(state:str)->None:
    match state:
        case "open":
            GPIO.output(SERVO2_PIN, GPIO.HIGH)
        case "close":   
            GPIO.output(SERVO2_PIN, GPIO.LOW)
        case _:
            GPIO.output(SERVO2_PIN, GPIO.LOW)

def servo3state(state:str)->None:
    match state:
        case "open":
            GPIO.output(SERVO3_PIN, GPIO.HIGH)
        case "close":   
            GPIO.output(SERVO3_PIN, GPIO.LOW)
        case _:
            GPIO.output(SERVO3_PIN, GPIO.LOW)

def solenoidstate()->None:        
    GPIO.output(Solenoid_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(Solenoid_PIN, GPIO.LOW)
           

def main()->None:   
    while True:
        user_input = input("test:").lower()
        if user_input[0] == "e":
            return
        match user_input:
            case "solenoid":
                solenoidstate()

            case "s1 0":
                servo1state("0")
            case "s1 1":
                servo1state("1")
            case "s1 2":
                servo1state("2")
            case "s1 3":
                servo1state("3")

            case "s2 open":
                servo2state("open")
            case "s2 close":
                servo2state("close")

            case "s3 open":
                servo3state("open")
            case "s3 close":
                servo3state("close")

if __name__ == "__main__":
    main()