#!/usr/bin/env python3

from pynput import keyboard
import rospy
from std_msgs.msg import Float64MultiArray, Float64

# BCM 
PIN_MOTOR_CW = 26
PIN_MOTOR_PWM = 18
PIN_SERVO_PULSE = 16
key_set={'w', 'a', 's', 'd'}
steering_angle = 90

def run_forward():
    motor_msgs = Float64MultiArray(data=[1, 100])
    motor_pub.publish(motor_msgs)

def run_backward():
    motor_msgs = Float64MultiArray(data=[0, 100])
    motor_pub.publish(motor_msgs)

def steering_to_left():
    servo_pub.publish(110)

def steering_to_right():
    servo_pub.publish(75)

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(key.char))
    except AttributeError:
        print('special key {0} pressed'.format(key))
    if key.char in key_set:
        if key.char == 'w':
            run_forward()
        elif key.char == 's':
            run_backward()
        elif key.char == 'a':
            steering_to_left()
        elif key.char == 'd':
            steering_to_right()
       #elif key.char == 'p':
       #     exit(0)
        


def on_release(key):
    print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        return False
    if key.char =='w' or key.char =='s':
        motor_msgs = Float64MultiArray(data=[1, 0])
        motor_pub.publish(motor_msgs)
    elif key.char =='a' or key.char =='d':
        servo_pub.publish(90)


rospy.init_node('teleop_ros')
motor_pub=rospy.Publisher('/rugged_car/basics/motor_pwm_direction_dutycycle', Float64MultiArray, queue_size=1)
servo_pub=rospy.Publisher('/rugged_car/basics/servo_angle_degree', Float64, queue_size=1)

with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()






