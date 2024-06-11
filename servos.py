import RPi.GPIO as GPIO
import time

SERVO1_PIN1 = 17
SERVO1_PIN2 = 4
SERVO2_PIN = 14
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO1_PIN1, GPIO.OUT)
GPIO.setup(SERVO1_PIN2, GPIO.OUT)
GPIO.setup(SERVO2_PIN, GPIO.OUT)

def servo1state(state:str):
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
                       
def servo2state(state:str):
    match state:
        case "open":
            GPIO.output(SERVO2_PIN, GPIO.HIGH)
        case "close":   
            GPIO.output(SERVO2_PIN, GPIO.LOW)

while True:
    servo1state("0")
    time.sleep(1)
    servo1state("1")
    time.sleep(1)
    servo1state("2")
    time.sleep(1)
    servo1state("3")
    # servo2state("close")