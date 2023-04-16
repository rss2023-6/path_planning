#!/usr/bin/env python

import rospy
import numpy as np
# from geometry_msgs.msg import PoseStamped, PoseArray
from geometry_msgs.msg import Vector3, Point, Pose, PoseStamped, PoseArray, Quaternion, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
from tf.transformations import euler_from_quaternion
from Queue import PriorityQueue, Queue

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

        #rviz buttons 
        # self.init_pose = rospy.Subscriber("/initialpose", PoseStamped, self.goal_cb, queue_size=10)
        # self.published_pt = rospy.Subsxcriber("/clicked_point", PoseStamped, self.goal_cb, queue_size=10)

        self.initialized = False
        self.map_grid = None
        self.map_resolution = None
        self.map_orientation = None
        self.map_rows = None
        self.map_cols = None
        self.map_origin = None
        self.current_pos = None

    # def transformation_matrix(self, th):
    #     return np.array([[np.cos(th), -np.sin(th), 0],
    #                         [np.sin(th), np.cos(th), 0],
    #                         [0, 0, 1]])
    
    # def px_2_m(self, map_resolution, px):
    #     return px*float(map_resolution)*self.lidar_scale_to_map_scale

    def m_2_px(self, m):
        return m/(float(self.map_resolution)*self.lidar_scale_to_map_scale)

    def map_cb(self, msg): ######
        self.map_resolution = msg.info.resolution
        self.map_orientation = msg.info.origin.orientation #quaternion
        self.map_origin = msg.info.origin.position

        #discretize map 
        self.map_rows = msg.info.height
        self.map_cols = msg.info.width
        map = np.array(msg.data).reshape((self.map_rows, self.map_cols)) #rows, cols
        self.map_grid = np.where(np.logical_and(0 <= map, map < 0.5), map, 1) #set to 1 if negative or greater than 0.5 aka occupied or unknown
       

    def odom_cb(self, msg): ######
        q = msg.pose.pose.orientation #not sure if we need this 
        roll, pitch, th = euler_from_quaternion()
        self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, th])
        self.initialized = True 

    def goal_cb(self, msg): ######
        start_pt = self.current_pos
        goal_pt = msg.pose.pose.position.x, msg.pose.pose.position.y

        start_px = start_pt #self.m_2_px(start_pt)
        end_px = goal_pt #self.m_2_px(goal_pt)

        self.plan_path(self, start_px, end_px, self.map_grid)

    def plan_path(self, start_point, end_point, map): ######
        ## CODE FOR PATH PLANNING ##

        path = self.a_star(start_point, end_point, map)
        if path:
            self.trajectory.points  = path

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


    def map_cost(self, current, next):
        '''calculates distance'''
        dist = np.sqrt((next[0]-current[0])**2 + (next[1]-current[1])**2)
        return dist

    def dist_heuristic(self, goal, next):
        dist = np.sqrt((goal[0]-next[0])**2 + (goal[1]-next[1])**2)
        return dist  

    def neighbors(self, pt):
        results = set()
        for dx, dy in np.meshgrid([-1,0,1],[-1,0,1]):
            nx, ny = pt[0] + dx, pt[1] + dy #indexing confusion
            if (0<= nx < self.map_rows) and (0<= ny < self.map_cols):
                if self.map([nx, ny]) == 0:
                    results.add((nx, ny))
        return results

    def a_star(self, init, goal, map):
        frontier = PriorityQueue() 
        frontier.put(init, 0)

        came_from = dict()
        cost_so_far = dict()

        came_from[init] = None
        cost_so_far[init] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == goal: #close to end point
                #get path 
                path = [goal]
                node = goal 
                while node != init:
                    node = came_from[node]
                    path.append(node)
                return list(reversed(path))
        
            for next in self.neighbors(current):
                new_cost = cost_so_far[current] + self.map_cost(current, next) #distance 
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.dist_heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current  

        return None #"path not found"  


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
