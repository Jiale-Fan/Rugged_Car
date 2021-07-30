#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time

rospy.init_node("test_path_publisher")

x=[0.0, 5.0, 5.0, 2.5, 3.0]
y=[0.0, 0.0, -1.5, -1.0, 0.0]

path_pub=rospy.Publisher("/rugged_car/tracking/path", Path, queue_size=10)

pm = Path()
pm.header.frame_id = "/camera_odom_frame"

for i in range(len(x)):
	ps = PoseStamped()
	ps.header.frame_id = "/camera_odom_frame"
	ps.pose.position.x = x[i]
	ps.pose.position.y = y[i]
	ps.pose.position.z = 0
	pm.poses.append(ps)

while 1:
	path_pub.publish(pm)
	time.sleep(1)

print("publish done!")
