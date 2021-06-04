#!/usr/bin/env python

from pynput import keyboard
import sys
import RPi.GPIO as GPIO
import time

PIN_MOTOR_CW = 26
PIN_MOTOR_PWM = 18
PIN_SERVO_PULSE = 16
key_set={'w', 'a', 's', 'd', 'p'}
GPIO.setmode(GPIO.BCM)

def run_forward():
    GPIO.output(PIN_MOTOR_PWM, GPIO.HIGH)
    GPIO.output(PIN_MOTOR_CW, GPIO.HIGH)

def run_backward():
    GPIO.output(PIN_MOTOR_PWM, GPIO.HIGH)
    GPIO.output(PIN_MOTOR_CW, GPIO.LOW)

def turn_right():
    for i in range(0,5):
        GPIO.output(PIN_SERVO_PULSE, GPIO.HIGH)
        time.sleep(float(70)/180*0.002+0.0005)
        GPIO.output(PIN_SERVO_PULSE, GPIO.LOW)
        time.sleep(0.02-(float(70)/180*0.002+0.0005))

def turn_left():
    for i in range(0,5):
        GPIO.output(PIN_SERVO_PULSE, GPIO.HIGH)
        time.sleep(float(110)/180*0.002+0.0005)
        GPIO.output(PIN_SERVO_PULSE, GPIO.LOW)
        time.sleep(0.02-(float(110)/180*0.002+0.0005))

def stop():
    GPIO.output(PIN_MOTOR_PWM, GPIO.LOW)

def direction_straight():
    for i in range(0,5):
        GPIO.output(PIN_SERVO_PULSE, GPIO.HIGH)
        time.sleep(float(90)/180*0.002+0.0005)
        GPIO.output(PIN_SERVO_PULSE, GPIO.LOW)
        time.sleep(0.02-(float(90)/180*0.002+0.0005))


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
            turn_left()
        elif key.char == 'd':
            turn_right()
        elif key.char == 'p':
            GPIO.cleanup()
            exit(0)


def on_release(key):
    print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        return False
    if key.char == 'w' or key.char == 's':
        stop()
    elif key.char == 'a' or key.char == 'd':
        direction_straight()

while True:

    GPIO.setup(PIN_SERVO_PULSE, GPIO.OUT)
    GPIO.setup(PIN_MOTOR_CW, GPIO.OUT)
    GPIO.setup(PIN_MOTOR_PWM, GPIO.OUT)
    GPIO.output(PIN_MOTOR_PWM, GPIO.LOW)

    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        listener.join()





