import RPi.GPIO as GPIO
import time

motor_pin = 3 
counter = 0 

def my_callback(channel):  # 边缘检测回调函数，详情在参见链接中
    global counter  # 设置为全局变量
    if GPIO.event_detected(motor_pin):  # 检测到一个脉冲则脉冲数加1
        counter = counter+1
        print("pulse: "+str(counter))


# 霍尔脉冲读取函数
GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor_pin, GPIO.IN)  # 
 
GPIO.add_event_detect(
    motor_pin, GPIO.RISING, callback=my_callback, bouncetime = 5)  # 在引脚上添加上升临界值检测再回调
    
try:
    while 1:
        time.sleep(0.2)
        # print(counter)
        pass
except KeyboardInterrupt:
    print(counter)
    print('exit')
