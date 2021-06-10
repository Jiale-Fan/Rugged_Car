#!/usr/bin/env python3

from pynput import keyboard
import rospy
from std_msgs.msg import Float64MultiArray, Float64

# BCM 
PIN_MOTOR_CW = 26
PIN_MOTOR_PWM = 18
PIN_SERVO_PULSE = 16
key_set={'w', 'a', 's', 'd', 'p'}
steering_angle = 90

def run_forward():
    control_pub(90, 1, 100)

def run_backward():
    control_pub(90, 0, 100)

def steering_to_left():
    control_pub(110, 1, 100)

def steering_to_right():
    control_pub(70, 1, 100)

def stop():
    control_pub(90, 1, 0)

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
        elif key.char == 'p':
            exit(0)


def on_release(key):
    print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        return False
    if key.char in key_set:
        stop()

def control_pub(angle, direction, motor_percentage):
    servo_pub.publish(angle)
    motor_msgs = Float64MultiArray(data=[direction, motor_percentage])
    motor_pub.publish(motor_msgs)

rospy.init_node('teleop_ros')
motor_pub=rospy.Publisher('/rugged_car/basics/motor_pwm_direction_dutycycle', Float64MultiArray, queue_size=1)
servo_pub=rospy.Publisher('/rugged_car/basics/servo_angle_degree', Float64, queue_size=1)

with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()






