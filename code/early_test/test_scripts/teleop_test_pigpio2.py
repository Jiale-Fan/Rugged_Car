#!/usr/bin/env python3

from pynput import keyboard
import sys
import pigpio
import time

# BCM 
PIN_MOTOR_CW = 26
PIN_MOTOR_PWM = 18
PIN_SERVO_PULSE = 16
key_set={'w', 'a', 's', 'd', 'p'}
steering_angle = 90

def run_forward():
    pi.write(PIN_MOTOR_PWM, 1)
    pi.write(PIN_MOTOR_CW, 1)

def run_backward():
    pi.write(PIN_MOTOR_PWM, 1)
    pi.write(PIN_MOTOR_CW, 0)

def steering_to_left():
    global steering_angle
    if steering_angle<=90:
        steering_angle+=20
        steering_command(steering_angle)

def steering_to_right():
    global steering_angle
    if steering_angle>=90:
        steering_angle-=20
        steering_command(steering_angle)

def stop():
    pi.write(PIN_MOTOR_PWM, 0)
        
def steering_command(angle):
    pi.set_PWM_dutycycle(PIN_SERVO_PULSE,angle*20/180+5)


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
            GPIO.cleanup()
            exit(0)


def on_release(key):
    print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        return False
    if key.char == 'w' or key.char == 's':
        stop()


pi = pigpio.pi()
pi.set_PWM_frequency(PIN_SERVO_PULSE, 50)
pi.set_PWM_range(PIN_SERVO_PULSE, 200) 
pi.set_PWM_dutycycle(PIN_SERVO_PULSE, 90*20/180+5)

with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()






