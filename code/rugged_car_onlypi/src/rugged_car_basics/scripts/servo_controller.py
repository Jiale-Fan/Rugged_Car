#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import pigpio

PIN_SERVO_PULSE = 16

class ServoController:
    def __init__(self):
        self.sub = rospy.Subscriber(
            '/rugged_car/basics/servo_angle_degree', Float64, self.servo_control, queue_size=1)
        self.pi = pigpio.pi()
        self.pi.set_PWM_frequency(PIN_SERVO_PULSE, 50)
        self.pi.set_PWM_range(PIN_SERVO_PULSE, 200)
        self.pi.set_PWM_dutycycle(PIN_SERVO_PULSE, 90*20/180+5)

    def servo_control(self, msg):
        if msg.data >= 0 and msg.data <= 180:
            self.pi.set_PWM_dutycycle(PIN_SERVO_PULSE, msg.data*20/180+5)


def main():
    rospy.init_node("servo", anonymous=True)
    sc = ServoController()
    rospy.loginfo("Servo controller started!")
    rospy.spin()

if __name__ == '__main__':
    main()
