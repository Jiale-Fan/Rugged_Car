#!/usr/bin/env python
import rospy
import tf
import sys
from nav_msgs.srv import GetMap
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
        self.plan_rx = []
        self.plan_ry = []

        self.tf = tf.TransformListener()
        self.goal_sub = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self.goalCallback)
        self.path_pub = rospy.Publisher(
            '/global_path', Path, queue_size=1)
        # self.map_sub = rospy.Subscriber(
        #     '/map', OccupancyGrid, self.mapCallback)
        self.updateMap()
        self.initPlanner()
        # self.updateGlobalPose()

    # def goalCallback(self, msg):
    #     self.plan_goal = msg
    #     self.plan_gx = msg.pose.position.x
    #     self.plan_gy = msg.pose.position.y
    #     # print("get new goal!!! ",self.plan_goal)
    #     self.replan(0)

    def goalCallback(self, msg):
        self.plan_goal = msg
        self.plan_gx = msg.pose.position.x
        self.plan_gy = msg.pose.position.y
        # print("get new goal!!! ",self.plan_goal)
        self.replan(0)

    def collisionCallback(self, msg):
        self.replan(0)

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
        # TODO get planner result
        rospy.loginfo('start: (%.2f, %.2f), target: (%.2f, %.2f), distance: %.2f' % (self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy, ((self.plan_sx-self.plan_gx)**2+(self.plan_sy-self.plan_gy)**2)**0.5))
        ####################
        self.plan_rx, self.plan_ry=self.planner_astar.planning(
            self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy)
        # self.plan_rx, self.plan_ry=self.planner_jps.planning(
        #     self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy)
        # self.plan_rx, self.plan_ry=self.planner_rrt.planning(
        #     self.plan_sx, self.plan_sy, self.plan_gx, self.plan_gy)
        ####################
        self.publishPath()
        res=True

    def initPlanner(self):
        map_data=np.array(self.map.data).reshape(self.map.info.width, -1)
        # map_data=np.array(self.map.data).reshape(-1, self.map.info.width)
        ox, oy=np.nonzero(map_data > 50)
        # self.plan_ox=(ox*self.map.info.resolution +
        #                 self.map.info.origin.position.x).tolist()
        # self.plan_oy=(oy*self.map.info.resolution +
        #                 self.map.info.origin.position.y).tolist()
        # TODO init your planner
        # e.g. self.planner = Planner(...)
        # self.planner = JPS_planner.JPSPlanner(map_data, self.map.info.resolution, (self.map.info.origin.position.x,
        # self.map.info.origin.position.y))
        # self.planner_astar=Astar_planner.AStarPlanner(
        #     self.map.info, ox, oy, self.plan_robot_radius)

        self.planner_astar=Astar_planner.AStarPlanner(
            self.map.info, map_data, ox, oy)
        # self.planner_jps=JPS_planner.JPSPlanner(
        #     self.map.info, ox, oy, self.plan_robot_radius)
        # self.planner_rrt=InformedRRTStar(self.map.info, ox, oy, self.plan_robot_radius, start=[self.plan_sx, self.plan_sy], goal=[
        #                                  self.plan_gx, self.plan_gy], randArea=[self.map.info.origin.position.x, self.map.info.origin.position.x+129*self.map.info.resolution])

        print('Planner initialized!')

    # def mapCallback(self, msg):
    #     self.map=msg
    #     pass

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
    pass


if __name__ == '__main__':
    main()
