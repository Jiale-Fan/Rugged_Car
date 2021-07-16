#!/usr/bin/env python
import rospy
import tf
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from pure_pursuit import *
from threading import Lock, Thread
import time

def main():
    rospy.init_node('path_publisher')
    test()
    pass


def test():
    rx = np.arange(0, 50, 0.5)
    ry = [math.sin(ix / 5.0) * ix / 2.0 for ix in rx]
    path = Path()
    path.header.seq = 0
    path.header.stamp = rospy.Time(0)
    path.header.frame_id = 'map'
    for i in range(len(rx)):
        pose = PoseStamped()
        pose.header.seq = i
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = 'map'
        pose.pose.position.x = rx[i]
        pose.pose.position.y = ry[i]
        pose.pose.position.z = 0.01
        pose.pose.orientation.x = 0  # self.rot[0]
        pose.pose.orientation.y = 0  # self.rot[1]
        pose.pose.orientation.z = 0  # self.rot[2]
        pose.pose.orientation.w = 1  # self.rot[3]
        path.poses.append(pose)
    pub = rospy.Publisher('/global_path', Path, queue_size=10)
    time.sleep(0.5)
    pub.publish(path)
    print("publish path!!!!!")
    # t.pathCallback(path)


if __name__ == '__main__':
    main()