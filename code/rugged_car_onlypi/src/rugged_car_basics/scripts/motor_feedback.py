#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import pigpio

PIN_DIRECT = 23
PIN_VEL = 24

class MotorFeedback:
    def __init__(self):
        self.pub = rospy.Publisher(
            '/rugged_car/basics/motor_velocity', Float64, queue_size=1)
        self.pi = pigpio.pi()
        self.pi.set_mode(PIN_DIRECT, pigpio.INPUT)
        self.pi.set_mode(PIN_VEL, pigpio.INPUT)
        self.direction = 1
        self.velocity = 0.0
        
    def get_direction(self):
        d = self.pi.read(PIN_DIRECT)
        if d == 0:
            self.derection = -1
        else:
            self.direction = 1
        
    def get_velocity(self):
        pass
        
    def pub_vel(self):
        self.get_direction()
        self.get_velocity()
        v = Float64(self.velocity*self.direction)
        self.pub.publish(v)
        
def main():
    rospy.init_node("feedback", anonymous=True)
    mf = MotorFeedback()
    rate = rospy.Rate(10)
    rospy.loginfo("motor feedback begin!")
    while not rospy.is_shutdown():
        mf.pub_vel()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
