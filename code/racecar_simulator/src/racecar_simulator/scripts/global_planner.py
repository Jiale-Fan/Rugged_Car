#!/usr/bin/env python
import rospy
import tf
import sys
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import OccupancyGrid
# from std_msgs.msg import Bool
import numpy as np
# import matplotlib.pyplot as plt
import Astar_planner


class GlobalPlanner:
    def __init__(self):
        self.plan_sx = 0.0
        self.plan_sy = 0.0
        self.plan_gx = 8.0
        self.plan_gy = -8.0
        self.plan_robot_radius = 0.325 #TODO Measure
        self.plan_ox = []
        self.plan_oy = []
        self.plan_rx = []
        self.plan_ry = []

        self.tf = tf.TransformListener()
        self.goal_sub = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.goalCallback)
        self.path_pub = rospy.Publisher(
            '/global_path', Path, queue_size=1)
        self.updateMap()
        self.initPlanner()
        # self.updateGlobalPose()


    def goalCallback(self, msg):
        self.plan_goal = msg
        self.plan_gx = msg.pose.position.x
        self.plan_gy = msg.pose.position.y
        # print("get new goal!!! ",self.plan_goal)
        self.replan(0)

    # def goalCallback(self, msg):
    #     s=self.planner_astar.coordIsObstacle(msg.pose.position.x, msg.pose.position.y)
    #     # print(s)

    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform(
                "/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
            (self.trans, self.rot) = self.tf.lookupTransform(
                '/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        self.plan_sx = self.trans[0]
        self.plan_sy = self.trans[1]

    def replan(self, req):
        print('get request for replan!!!!!!!!')

        self.updateGlobalPose()
        # get planner result
        rospy.loginfo('start: (%.2f, %.2f), target: (%.2f, %.2f), distance: %.2f' % (self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy, ((self.plan_sx-self.plan_gx)**2+(self.plan_sy-self.plan_gy)**2)**0.5))
        ####################
        self.plan_rx, self.plan_ry=self.planner_astar.planning(
            self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy)
        ####################
        self.publishPath()

    def initPlanner(self):
        # !!!NOTE!!! the map orientation must be (0,0,0,1) to run correctly
        map_data=np.array(self.map.data).reshape(
            self.map.info.height, -1).transpose()
        # ox, oy=np.nonzero(map_data > 50)
        # self.plan_ox=(ox*self.map.info.resolution +
        #                 self.map.info.origin.position.x).tolist()
        # self.plan_oy=(oy*self.map.info.resolution +
        #                 self.map.info.origin.position.y).tolist()
        # init your planner
        self.planner_astar=Astar_planner.AStarPlanner(
            self.map.info, self.plan_robot_radius, map_data)

        rospy.loginfo('Planner initialized!')

    def mapCallback(self, msg):
        self.map=msg

    def updateMap(self):
        try:
            map=rospy.wait_for_message('/map', OccupancyGrid, 10)
        except:
            e=sys.exc_info()[0]
            print('No map received: %s' % e)
        # Update for planning algorithm
        self.map=map
        rospy.loginfo('Map received!')

    def publishPath(self):
        path=Path()
        path.header.seq=0
        path.header.stamp=rospy.Time(0)
        path.header.frame_id='map'
        for i in range(len(self.plan_rx)):
            pose=PoseStamped()
            pose.header.seq=i
            pose.header.stamp=rospy.Time(0)
            pose.header.frame_id='map'
            pose.pose.position.x=self.plan_rx[len(self.plan_rx)-1-i]
            pose.pose.position.y=self.plan_ry[len(self.plan_rx)-1-i]
            pose.pose.position.z=0.01
            pose.pose.orientation.x=0
            pose.pose.orientation.y=0
            pose.pose.orientation.z=0
            pose.pose.orientation.w=1
            path.poses.append(pose)
        self.path_pub.publish(path)


def main():
    rospy.init_node('global_planner')
    gp=GlobalPlanner()
    rospy.spin()


if __name__ == '__main__':
    main()
