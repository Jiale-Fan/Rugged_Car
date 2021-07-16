#!/usr/bin/env python3
from numpy.core.numeric import Inf
import functools
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
import time


class SearchBasedPlanner:
    def __init__(self, map_info, robot_radius, map_data):
        time.sleep(1)
        self.map_info = map_info
        self.sx = 0.0  # start_x
        self.sy = 0.0  # start_y
        self.gx = 0.0  # goal_x
        self.gy = 0.0  # goal_y
        self.resolution = map_info.resolution
        # self.r = int(robot_radius/self.resolution) + 1
        self.r = 0
        self.orign_x = map_info.origin.position.x
        self.orign_y = map_info.origin.position.y
        # self.ox = ox  # obstacle_x
        # self.oy = oy  # obstacle_y
        self.rx = []  # path_planed_x
        self.ry = []  # path_planed_y

        self.m = map_info.width
        self.n = map_info.height

        self.map = np.zeros((self.m, self.n))
        self.expanded_map = np.zeros((self.m, self.n))
        self.expanded_pub = rospy.Publisher(
            '/course_agv/expanded_pub', OccupancyGrid, queue_size=1)
        self.dilation_pub = rospy.Publisher(
            '/course_agv/dilation_pub', OccupancyGrid, queue_size=1)


        # self.map_init() # can be used to inflate the obstacles
        self.map=map_data
        self.publishObstacleDilation()

        print("resolution is "+str(self.resolution))
        print("width: "+str(self.m)+" height: "+str(self.n))
        print("origin:"+str((self.orign_x, self.orign_y)))

    def calCost(self, node, parent):
        if node == self.start:
            g = 0
        else:
            g = self.costs[parent][0] + \
                ((parent[0]-node[0])**2+(parent[1]-node[1])**2)**0.5
        h = ((self.target[0]-node[0])**2+(self.target[1]-node[1])**2)**0.5
        f = g+h
        # print((g, h, f))
        return (g, h, f)

    def compareCost(self, node_1, node_2):
        if self.costs[node_1][2] < self.costs[node_2][2]:
            return -1
        elif self.costs[node_1][2] > self.costs[node_2][2]:
            return 1
        else:
            return 0

    def coordIsObstacle(self, x, y):
        grid_x=int((x-self.orign_x)//self.resolution)
        grid_y=int((y-self.orign_y)//self.resolution)
        return self.isObstacle(grid_x, grid_y)

    def isObstacle(self, x, y=0):

        if type(x) == tuple or type(x) == list:
            y = x[1]
            x = x[0]

        # print('check position: ', x, y)

        if x >= 0 and y >= 0 and x < self.m and y < self.n:
            # print(self.map[x, y])
            if self.map[x, y] > 50 or self.map[x, y] < 0:
                # print(True)
                return True
            else:
                # print(False)
                return False
        else:
            # print(True)
            return True

    # def map_init(self):
    #     num = self.ox.shape
    #     num = num[0]
    #     for k in range(num):
    #         i = self.ox[k]
    #         j = self.oy[k]

    #         self.map[i, j] = int(1)
    #         # print(i,j)

    #         # for w in [(x-self.r) for x in range(2*self.r+1)]:
    #         #     temp = int(pow(pow(self.r, 2)-pow(w, 2), 0.5))
    #         #     for h in [(x-temp) for x in range(2*temp+1)]:
    #         #         if i+w in range(self.m) and j+h in range(self.n) and (pow(w, 2)+pow(h, 2)) <= pow(self.r, 2):
    #         #             self.map[i+w, j+h] = int(1)

    #     # print('touched')
    #     # np.savetxt("/home/edward/dets2.txt", self.map,fmt='%f',delimiter=',')
    #     self.publishObstacleDilation()

    def publishObstacleDilation(self):
        count = 0
        expanded_msg = OccupancyGrid()
        expanded_msg.header.seq = 0
        expanded_msg.header.stamp = rospy.Time(0)
        expanded_msg.header.frame_id = 'map'
        expanded_msg.info = self.map_info
        expanded_msg.data=self.map.transpose().reshape(1,-1).tolist()[0]
        for i in range(10):
            self.dilation_pub.publish(expanded_msg)
            time.sleep(0.2)
        rospy.loginfo("obstacle dilation published")
        return count
    
    def publishExpanded(self):
        count = 0
        expanded_msg = OccupancyGrid()
        expanded_msg.header.seq = 0
        expanded_msg.header.stamp = rospy.Time(0)
        expanded_msg.header.frame_id = 'map'
        expanded_msg.info = self.map_info
        expanded_msg.data=self.expanded_map.transpose().reshape(1,-1).tolist()[0]
        for i in range(10):
            self.expanded_pub.publish(expanded_msg)
            # time.sleep(0.2)
        rospy.loginfo("expanded map published")
        return count


class AStarPlanner(SearchBasedPlanner):
    def planning(self, plan_sx, plan_sy, plan_gx, plan_gy):

        start_time = rospy.Time.now()

        ####### initialization #######
        self.expanded_map=np.zeros((self.m, self.n))
        rx = []
        ry = []

        plan_sx = int((plan_sx-self.orign_x)//self.resolution)
        plan_sy = int((plan_sy-self.orign_y)//self.resolution)
        plan_gx = int((plan_gx-self.orign_x)//self.resolution)
        plan_gy = int((plan_gy-self.orign_y)//self.resolution)

        # start_point=Node(plan_sx, plan_sy)
        self.target = (plan_gx, plan_gy)
        self.start = (plan_sx, plan_sy)
        openlist = [self.start]  # element format: [(x,y), (g,h)]
        # self.closelist=[]

        # print("start: ")
        # print(self.start)
        # print("target: ")
        # print(self.target)

        # key is the node, corresponding value is the coordinate of its parent
        self.parent = dict()
        self.costs = dict()  # key is the node, corresponding value is the (g,h,f) of its parent
        self.costs[self.start] = self.calCost(self.start, self.start)

        ####### loop #######
        while(len(openlist) != 0):
            openlist.sort(key=functools.cmp_to_key(self.compareCost))
            # print(openlist)
            current = openlist.pop(0)

            if current == self.target:
                # print('Path found!')
                break

            neighbors = self.getNeighbors(current)
            # print(neighbors)
            for node in neighbors:
                if node not in self.parent:   # not in openlist or closelist
                    self.parent[node] = current  # set the parent
                    cost = self.calCost(node, current)
                    openlist.append(node)   # add to openlist
                    self.costs[node] = cost   # record the cost
                    # print(node)
                else:  # in openlist or closelist, check if it's necessary to update
                    cost = self.calCost(node, current)
                    if cost[2] < self.costs[node][2]:
                        self.parent[node] = current  # reset the parent
                        self.costs[node] = cost

        # publish expanded area
        expanded_num=self.publishExpanded()

        ####### path generation #######
        if self.target in self.parent:
            path = [self.target]
            idx_x = [self.target[0]]
            idx_y = [self.target[1]]
            current = self.parent[self.target]
            while current != self.start:
                idx_x.append(current[0])
                idx_y.append(current[1])
                path.append(current)
                current = self.parent[current]
                # print(current)
            idx_x.append(self.start[0])
            idx_y.append(self.start[1])
            path.append(self.start)

            # print(path)
            idx_x = np.array(idx_x)
            idx_y = np.array(idx_y)

            rx = (idx_x+0.5)*self.resolution+self.orign_x
            ry = (idx_y+0.5)*self.resolution+self.orign_y

            end_time = rospy.Time.now()
            duration = end_time - start_time
            rospy.loginfo("A* search completed. Path found. Time consumed: %dms Node expanded: %d path distance: %.2f" % (
                duration.to_sec()*1000, expanded_num, self.costs[self.target][0]*self.resolution))

            return rx, ry
        else:
            end_time = rospy.Time.now()
            duration = end_time - start_time
            rospy.loginfo("A* search completed. No path found. Time consumed: %dms Node expanded: %d" % (
                duration.to_sec()*1000, expanded_num))
            return rx, ry

    def getNeighbors(self, node):
        self.expanded_map[node[0],node[1]] = 127
        # self.publishExpanded()
        # print(node)
        # a=input()

        neighbors = []
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                if not (i == 0 and j == 0):
                    if not self.isObstacle(int(node[0])+i, int(node[1]+j)):
                        neighbors.append((int(node[0])+i, int(node[1]+j)))

        return neighbors
