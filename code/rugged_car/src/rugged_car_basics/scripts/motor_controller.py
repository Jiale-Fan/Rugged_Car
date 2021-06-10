#!/usr/bin/env python3

######################################################################
# This script is undone. It's intended to implement the close-loop
# control of the motor. 


import rospy
from std_msgs.msg import Float64MultiArray, Float64
import pigpio

PIN_MOTOR_CW = 26
PIN_MOTOR_PWM = 18


class MotorController:
    def __init__(self):
        self.sub = rospy.Subscriber(
            '/rugged_car/basics/motor_pwm_direction_dutycycle', Float64MultiArray, self.motor_control, queue_size=1)
            # the first float indicates the direction(0, 1) and the second float indicates dutycycle(0~100)
        self.pi = pigpio.pi()
        self.pi.set_PWM_frequency(PIN_MOTOR_PWM, 20000)
        self.pi.set_PWM_range(PIN_MOTOR_PWM, 1000)
        self.pi.set_PWM_dutycycle(PIN_MOTOR_PWM, 0)
        self.pi.write(PIN_MOTOR_CW, 1)

    def motor_control(self, msg):
        self.pi.write(PIN_MOTOR_CW, bool(msg.data[0]))
        self.pi.set_PWM_dutycycle(PIN_MOTOR_PWM, msg.data[1]*10)
        


def main():
    rospy.init_node("motor", anonymous=True)
    sc = MotorController()
    rospy.loginfo("Motor controller started!")
    rospy.spin()

if __name__ == '__main__':
    main()
