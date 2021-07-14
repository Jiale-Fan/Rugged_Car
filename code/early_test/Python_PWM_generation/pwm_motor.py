
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)

p = GPIO.PWM(12, 1000)  # 通道\频率
dc = 50
p.ChangeDutyCycle(dc)
p.start(0)
try:
    while 1:
        t = input()
        # print('got')
        if t == 'w':
            if dc + 10<=60:
                dc += 10
        elif t == 's':
            if dc - 10 >= 0:
                dc -= 10
        print(dc)
        p.ChangeDutyCycle(dc)
except KeyboardInterrupt:
    pass
p.stop()
GPIO.cleanup()
print('program exit')

