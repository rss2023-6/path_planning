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
from scipy.spatial.transform import Rotation as R

from scipy.misc import imread, imsave #scipy 1.2.0

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
        self.car_length = 0.5 #0.35
        self.car_box_length = None
        self.search_step_size = 5
        self.seen = set()

        self.init_time = None #rospy.get_rostime()
        self.final_time = None
        rospy.loginfo("planning initialized")


    def map_cb(self, msg): ######
        self.map_resolution = msg.info.resolution
        # rospy.loginfo("map_resolution")
        # rospy.loginfo(self.map_resolution)
        q = msg.info.origin.orientation
        roll, pitch, th = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # rospy.loginfo("euler_from_quaternion")
        # rospy.loginfo(euler_from_quaternion([q.x, q.y, q.z, q.w]))
        self.map_orientation = q#th 
        self.map_origin = msg.info.origin.position
        # self.WorldToMapTransform = np.array([[np.cos(-self.map_orientation), -np.sin(-self.map_orientation), -self.map_origin.x],
        #                               [np.sin(-self.map_orientation), np.cos(-self.map_orientation), -self.map_origin.y],
        #                               [0, 0, 1]])

        # rospy.loginfo("self.map_origin")
        # rospy.loginfo(self.map_origin)

        
        #discretize map 
        self.map_rows = msg.info.height
        self.map_cols = msg.info.width
        self.car_box_length = int(self.car_length/(self.map_resolution * 2))
        map = np.array(msg.data).reshape((self.map_rows, self.map_cols)) #rows, cols
        self.map_grid = np.where(np.logical_and(0 <= map, map < 0.5), 0, 1) #set to 1 if negative or greater than 0.5 aka occupied or unknown
        # imsave("/home/racecar/racecar_ws/src/path_planning/maps/occupancy2.png", (np.true_divide(self.map_grid, 100.0) * 255.0).T)
       

    def odom_cb(self, msg): ######
        #q = msg.pose.pose.orientation #not sure if we need this 
        #roll, pitch, th = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]) #np.array([-18.0, -9.0])#
        self.initialized = True 

    def goal_cb(self, msg): ######
        if self.initialized:
            start_pt = self.current_pos
            goal_pt = msg.pose.position.x, msg.pose.position.y #(6,18) #
            self.init_time = rospy.get_rostime()

            # rospy.loginfo("start and end in world")
            # rospy.loginfo(start_pt)
            # rospy.loginfo(goal_pt)

            start_px = (self.world_to_pixel_frame(start_pt[0],start_pt[1]))#change to map frame 
            end_px = (self.world_to_pixel_frame(goal_pt[0],goal_pt[1]))  #change to map frame 

            # rospy.loginfo("start and end in map")
            # rospy.loginfo(start_px)
            # rospy.loginfo(end_px)

            # rospy.loginfo("back to world")
            # rospy.loginfo(self.map_to_world_frame(start_px[0],start_px[1]))
            # rospy.loginfo(self.map_to_world_frame(end_px[0],end_px[1]))

            self.plan_path(start_px, end_px, self.map_grid)
        else:
            rospy.loginfo("start_pt not init")

    def plan_path(self, start_point, end_point, map): ######
        ## CODE FOR PATH PLANNING ##

        path = self.a_star(start_point, end_point, map)

        if path:
            self.final_time = rospy.get_rostime()
            rospy.loginfo("time duration")
            diff = self.final_time - self.init_time
            rospy.loginfo(diff.to_sec())
            #change map frame back to world frame 
            rospy.loginfo("path")
            rospy.loginfo(self.total_distance(path))
            
            world_path = []
            for tup in path:
                world_path.append(self.pixel_to_world_frame(tup[0],tup[1]))
            rospy.loginfo("world_path")
            rospy.loginfo(self.total_distance(world_path))
            
            self.trajectory.points  = world_path

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz(60)

    def total_distance(self, path):
        sum = 0
        for i in range(len(path)-1):
            sum += self.map_cost(path[i], path[i+1])
        return sum

    def map_cost(self, current, next):
        '''calculates distance'''
        dist = np.sqrt((next[0]-current[0])**2 + (next[1]-current[1])**2)
        return dist

    def dist_heuristic(self, goal, next):
        dist = np.sqrt((goal[0]-next[0])**2 + (goal[1]-next[1])**2)
        return dist  
    
    # def world_to_map_frame(self, nx, ny):
    #     norm_point = np.array([nx, ny, 1])
    #     px_point = np.matmul(self.WorldToMapTransform, norm_point)/self.map_resolution
    #     return (px_point[0].astype("uint8"), px_point[1].astype("uint8"))
    
    # def map_to_world_frame(self, nx, ny):
    #     norm_point = np.array([nx * self.map_resolution, ny * self.map_resolution, 1])
    #     MapToWorldTransform = np.linalg.inv(self.WorldToMapTransform)
    #     world_pt = np.matmul(MapToWorldTransform, norm_point)

    #     return (world_pt[0], world_pt[1])

    def world_to_pixel_frame(self, nx, ny):
        real_pos = np.array([nx, ny, 1]).T
        q = [self.map_orientation.x, self.map_orientation.y, self.map_orientation.z, self.map_orientation.w]
        rot = R.from_quat(q)
        rot_pix_to_world = rot.as_dcm()
        rot_pix_to_world[0,2] = self.map_origin.x
        rot_pix_to_world[1,2] = self.map_origin.y

        rot_world_to_pixel =  np.linalg.inv(rot_pix_to_world)

        px_point = np.matmul(rot_world_to_pixel, real_pos)/self.map_resolution
        return (px_point[0].astype(int), px_point[1].astype(int))
    
    def pixel_to_world_frame(self, nx, ny):
        pix_pos = np.array([nx * self.map_resolution, ny * self.map_resolution, 1]).T
        q = [self.map_orientation.x, self.map_orientation.y, self.map_orientation.z, self.map_orientation.w]
        rot = R.from_quat(q)
        rot_pix_to_world = rot.as_dcm()
        rot_pix_to_world[0,2] = self.map_origin.x
        rot_pix_to_world[1,2] = self.map_origin.y

        real_point = np.matmul(rot_pix_to_world, pix_pos)
        return (real_point[0], real_point[1])

    
    def get_space_around_car(self, map_coords):
        map_x = int(map_coords[0])
        map_y = int(map_coords[1])
        
        car_slice = self.map_grid[map_y - self.car_box_length : map_y + self.car_box_length, map_x - self.car_box_length : map_x + self.car_box_length]
        
        total_occupancy = np.sum(car_slice, axis=None)
        # rospy.loginfo(total_occupancy)
        if total_occupancy != 0:
            # rospy.loginfo(map_x)
            # rospy.loginfo(map_y)
            # rospy.loginfo(car_slice)
            rospy.loginfo(total_occupancy)
        return False if total_occupancy != 0 else True 

    def neighbors(self, pt):
        results = set()
        #xv, yv = np.meshgrid([-self.search_step_size, 0, self.search_step_size], [-self.search_step_size, 0, self.search_step_size], indexing='ij')
        for i in [-self.search_step_size, 0, self.search_step_size]: #range(3):
            for j in [-self.search_step_size, 0, self.search_step_size]:#in range(3):
                if (i != 0) and (j !=0):
                    #nx, ny = pt[0] + xv[i,j], pt[1] + yv[i,j] #indexing confusion
                    nx, ny = pt[0] + i, pt[1] + j
                    map_coords = (nx, ny)
                    if (nx, ny) not in self.seen:
                        self.seen.add((nx, ny))
                        if self.get_space_around_car(map_coords):
                            results.add((nx, ny))
                            # rospy.loginfo("results added")
                            # rospy.loginfo((nx, ny))
        return results

    def a_star(self, init, goal, map):
        frontier = PriorityQueue() 
        frontier.put((0, init))

        came_from = dict()
        cost_so_far = dict()

        came_from[init] = None
        cost_so_far[init] = 0

        while not frontier.empty():
            
            # rospy.loginfo(frontier.get())
            current = frontier.get()[1]

            if current in came_from.keys():
                pass

            if self.dist_heuristic(current, goal) < self.search_step_size: #close to end point
                #get path 
                path = [current]
                node = current
                while node != init:
                    node = came_from[node]
                    path.append(node)
                return list(reversed(path))
        
            for next in self.neighbors(current):
                new_cost = cost_so_far[current] + self.map_cost(current, next) #distance 
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.dist_heuristic(goal, next)
                    frontier.put((priority, next))
                    came_from[next] = current  

        rospy.loginfo("no path found")
        return None #"path not found"  


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
