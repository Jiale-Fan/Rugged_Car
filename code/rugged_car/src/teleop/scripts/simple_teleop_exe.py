#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import RPi.GPIO as GPIO
import time

PIN_MOTOR_CW = 26
PIN_MOTOR_PWM = 18
PIN_SERVO_PULSE = 16
key_set={'w', 'a', 's', 'd', 'p'}
GPIO.setmode(GPIO.BCM)
steering_angle = 90

def run_forward():
    GPIO.output(PIN_MOTOR_PWM, GPIO.HIGH)
    GPIO.output(PIN_MOTOR_CW, GPIO.HIGH)

def run_backward():
    GPIO.output(PIN_MOTOR_PWM, GPIO.HIGH)
    GPIO.output(PIN_MOTOR_CW, GPIO.LOW)

def steering_to_left():
    steering_command(110)

def steering_to_right():
    steering_command(70)

def stop():
    GPIO.output(PIN_MOTOR_PWM, GPIO.LOW)
        
def steering_command(angle):
    for i in range(0,5):
        GPIO.output(PIN_SERVO_PULSE, GPIO.HIGH)
        time.sleep(float(angle)/180*0.002+0.0005)
        GPIO.output(PIN_SERVO_PULSE, GPIO.LOW)
        time.sleep(0.02-(float(angle)/180*0.002+0.0005))

def steering_straight():
    steering_command(90)

def teleop_execute(msg):
    speed = msg.drive.speed
    angle = msg.drive.steering_angle
    if speed > 0:
        run_forward()
    elif speed < 0:
        run_backward()
    if angle>0:
        steering_to_left()
    elif angle<0:
        steering_to_right()
    else:
        steering_straight()
        

if __name__ == "__main__":
    GPIO.setup(PIN_SERVO_PULSE, GPIO.OUT)
    GPIO.setup(PIN_MOTOR_CW, GPIO.OUT)
    GPIO.setup(PIN_MOTOR_PWM, GPIO.OUT)
    GPIO.output(PIN_MOTOR_PWM, GPIO.LOW)
    sub=rospy.Subscriber('/ackermann_cmd_mux/input/teleop',AckermannDriveStamped,teleop_execute)
    rospy.spin()
