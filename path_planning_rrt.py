#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
from scipy import ndimage
from scipy.misc import imread, imsave
import rospkg
import time, os
import random
from utils import LineTrajectory

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        #self.odom_topic = rospy.get_param("~odom_topic")
        self.start_point = None
        self.end_point = None
        self.map = None
        self.bounds = None
        self.radius = None
        self.resolution = None
        self.map_orientation = None
        self.map_origin = None
        self.WorldToMapTransform = None
        self.ndimage_dilation = 3 #ndimage, 15

        self.odom_topic = "/odom"
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        self.test_pub = rospy.Publisher("/test", Pose, queue_size=10)
        


    def map_cb(self, msg):
        '''Turn map OccupancyGrid msg into a numpy array'''
        
        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width)) #rows, cols
        #dilation
        # img_map = np.true_divide(map, 100.0) * 255.0
        # dilated_img = dilation(img_map, disk(self.dilation_disk))
        # map = np.true_divide(dilated_img, 255.0) * 100.0
        self.map = ndimage.binary_dilation(self.map, iterations = self.ndimage_dilation)
        rospy.loginfo("dilation true")

        imsave("/home/racecar/racecar_ws/src/path_planning/maps/occupancy.png", (np.true_divide(self.map, 100.0) * 255.0).T)
        #self.map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        
        self.bounds = (msg.info.width, msg.info.height)
        self.radius = int(.5/msg.info.resolution)
        self.resolution = msg.info.resolution

        roll, pitch, th = euler_from_quaternion([msg.info.origin.orientation.w, msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z])
        self.map_orientation = th
        self.map_origin = msg.info.origin.position
        self.WorldToMapTransform = np.array([[np.cos(-1.*self.map_orientation), np.sin(-1.*self.map_orientation), -1.*self.map_origin.x],
                                      [np.sin(-1.*self.map_orientation), np.cos(-1.*self.map_orientation), -1.*self.map_origin.y],
                                      [0, 0, 1]])

        rospy.loginfo("Map received")

    def odom_cb(self, msg):
        if self.WorldToMapTransform is None:
            return
        self.start_point = self.transform_world_to_map((msg.pose.pose.position.x, msg.pose.pose.position.y))
        #rospy.loginfo("Odom received")

    def goal_cb(self, msg):
        
        self.end_point = self.transform_world_to_map((msg.pose.position.x, msg.pose.position.y))
        rospy.loginfo("Goal received")
        self.plan_path(self.start_point, self.end_point, self.map)
        rospy.loginfo("Path Planned")

    def plan_path(self, start_point, end_point, map):
        '''Implement rrt'''
        ## CODE FOR PATH PLANNING ##
        rospy.loginfo(start_point)
        start_node = Node(start_point, None, 0)
        points = [start_node]
        zero_indices = np.argwhere(map == 0)
        while True:
            random_index = zero_indices[np.random.randint(len(zero_indices))]
            #testpoint = tuple(random_index)
            #rospy.loginfo(distance(testpoint, points[-1].location))
            testpoint = (random.randint(0,self.bounds[0]-1), random.randint(0,self.bounds[1]-1))


            closestdist = np.inf
            closestpoint = (0,0)
            for index, node in enumerate(points):
                testdist = self.distance(testpoint, node.location)
                if testdist < closestdist:
                    closest = node
                    closestdist = testdist
            if not self.collisioncheck(self.map, closest.location, testpoint, self.radius):
                goodnode = Node(testpoint, closest, closest.cost+closestdist)
                points.append(goodnode)
                if self.goalcheck(end_point, testpoint, self.radius):
                    break
                
        nodepath = [points[-1]]
        count = 0
        while True:
            if nodepath[count] == None:
                break
            else:
                nodepath.append(nodepath[count].parent)
                count+=1
        nodepath.reverse()
        nodepath = nodepath[1:]

        nodepathmod = []

        path = []
        rospy.loginfo(len(nodepath))
        nodeindex = 0
        while nodeindex < len(nodepath)-1:
            for nodeindex2 in range(nodeindex, len(nodepath)):
                if self.collisioncheck(self.map, nodepath[nodeindex].location, nodepath[nodeindex2].location, self.radius):
                    rospy.loginfo("appending")
                    nodepathmod.append(nodepath[nodeindex2-1])
                    nodeindex = nodeindex2-1
                    break
                elif nodeindex2 == len(nodepath)-1:
                    nodepathmod.append(nodepath[nodeindex2])
                    nodeindex = nodeindex2
                    break
                else:
                    rospy.loginfo(nodeindex2)
                    nodeindex = nodeindex2

        for node in nodepath:
            path.append(self.transform_map_to_world((node.location[0], node.location[1])))
       
        rospy.loginfo(path)
        self.trajectory.points = path

    
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz(60)

    def distance(self, p1, p2):
        return ((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)**.5

    def goalcheck(self, goal, test, radius):
        return self.distance(goal, test) < radius
        
    def bresenham_line(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return points

    def collisioncheck(self, map, start, test, radius):
        line_points = self.bresenham_line(start[0], start[1], test[0], test[1])

        for point in line_points:
            x, y = point
            if map[y][x] > 0:
                rospy.loginfo("collision")
                return True

        return False
            
    
    def dist_from_line(self, p, m, b):
        return abs(-m*p[0]+p[1]+b)/((m**2+1)**.5)
    
    def transform_world_to_map(self, point):
        norm_point = np.array([point[0], point[1], 1])

        transformed = np.matmul(self.WorldToMapTransform, norm_point)/self.resolution
        return (int(transformed[0]), int(transformed[1]))
    
    def transform_map_to_world(self, point):
        norm_point = np.array([point[0]*self.resolution, point[1]*self.resolution, 1])
        transformed =  np.matmul(np.linalg.inv(self.WorldToMapTransform), norm_point)
        return (transformed[0], transformed[1])

class Node:
    def __init__(self, location, parent, cost):
        self.location = location
        self.parent = parent
        self.cost = cost

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
