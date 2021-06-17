
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(5, GPIO.OUT)
GPIO.output(5,GPIO.HIGH)

try:
    while 1:
        pass
except KeyboardInterrupt:
    pass
GPIO.cleanup()
print('program exit')

