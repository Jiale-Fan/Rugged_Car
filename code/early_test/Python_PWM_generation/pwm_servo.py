
import sys
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.OUT)
 
GPIO.output(16, GPIO.HIGH)
time.sleep(float(sys.argv[1])/180*0.002+0.0005)
GPIO.output(16, GPIO.LOW)
GPIO.cleanup()

