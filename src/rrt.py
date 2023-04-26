#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion, rotation_matrix, concatenate_matrices
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
        self.ndimage_dilation = 10 #ndimage, 15

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
        rospy.loginfo(self.map.shape)
        rospy.loginfo("dilation true")

        imsave("/home/racecar/racecar_ws/src/path_planning/maps/occupancy.png", (np.true_divide(self.map, 100.0) * 255.0).T)
        #self.map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        
        self.bounds = (msg.info.width, msg.info.height)
        self.radius = int(.5/msg.info.resolution)
        self.resolution = msg.info.resolution
        self.map_orientation = msg.info.origin.orientation
        self.map_origin = msg.info.origin.position
        rospy.loginfo(self.map_origin)
        rospy.loginfo("Map received")

    def odom_cb(self, msg):
        if self.map_orientation is None:
            return
        self.start_point = self.world_to_pixel_frame(msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.start_point = (abs(self.start_point[0]), abs(self.start_point[1]))
        #rospy.loginfo((msg.pose.pose.position.x, msg.pose.pose.position.y))
        #rospy.loginfo(self.start_point)

    def goal_cb(self, msg):
        
        self.end_point = self.world_to_pixel_frame(msg.pose.position.x, msg.pose.position.y)
        self.end_point = (abs(self.end_point[0]), abs(self.end_point[1]))
        rospy.loginfo(self.end_point)
        rospy.loginfo(self.map[self.end_point[1], self.end_point[0]])
        rospy.loginfo("Goal received")
        self.plan_path(self.start_point, self.end_point, self.map)
        rospy.loginfo("Path Planned")

    def plan_path(self, start_point, end_point, map):
        '''Implement rrt'''
        ## CODE FOR PATH PLANNING ##
        start_node = Node(start_point, None, 0)
        points = [start_node]
        zero_indices = np.argwhere(map == False)
        while True:
            random_index = zero_indices[np.random.randint(len(zero_indices))]
            testpoint = tuple(random_index)
            testpoint = (testpoint[1], testpoint[0])
            rospy.loginfo(testpoint)
            #rospy.loginfo(self.map[testpoint[0], testpoint[1]])
            #rospy.loginfo(self.distance(testpoint, points[-1].location))
            #testpoint = (random.randint(0,self.bounds[0]-1), random.randint(0,self.bounds[1]-1))
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

########## code for pruning path ##########
        path = []
        
        current = 0
        nodepathmod = [nodepath[current]]
        while current != len(nodepath)-1:
            for i in range(current+1, len(nodepath)):
                if self.collisioncheck(self.map, nodepath[current].location, nodepath[i].location, self.radius):
                    current = i-1
                    break
                elif i == len(nodepath)-1:
                    current = i
                    break
                else:
                    continue
            nodepathmod.append(nodepath[current])
            
##########  end of code for pruning path ##########   

        for node in nodepathmod:
            path.append(self.pixel_to_world_frame(node.location[0], node.location[1]))
    
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
            if x < 0 or y < 0 or x >= map.shape[1] or y >= map.shape[0]:
                #rospy.loginfo("out of bounds")
                return True
            if map[y][x] == True:
                rospy.loginfo("collision")
                return True

        return False
            
    
    def dist_from_line(self, p, m, b):
        return abs(-m*p[0]+p[1]+b)/((m**2+1)**.5)
    
    def world_to_pixel_frame(self, nx, ny):
        mapx=nx-self.map_origin.x
        mapy=ny-self.map_origin.y
        pixx=mapx/self.resolution
        pixy=mapy/self.resolution
        return (int(pixx), int(pixy))
    def pixel_to_world_frame(self, nx, ny):
        pixx=-nx*self.resolution
        pixy=-ny*self.resolution
        mapx=pixx+self.map_origin.x
        mapy=pixy+self.map_origin.y
        return (mapx, mapy)
    
    def quaternion_rotation_matrix(self, Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix
    
    
class Node:
    def __init__(self, location, parent, cost):
        self.location = location
        self.parent = parent
        self.cost = cost

if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
