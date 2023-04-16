#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = 0.2 #filled in for testing purposes, please update, larger is more smooth and smaller is more oscillations
        self.speed            = 1 #filled in for testing purposes, please update
        self.wheelbase_length = 0.24 #flilled in for testing purposes, please update
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.n = len(self.trajectory.points)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

    def get_line_segments(self):
        pass

    def find_closest_point(self):
        pass

    def find_circle_intersection(self, index):
        radius = self.lookahead
        rx = self.x
        ry = self.y
        
        found = False
        #assume the index is the index of the line segment that the closest point lines on

        for i in range(index, n - 1):
            x0 = self.trajectory.points[i][0]
            y0 = self.trajectory.points[i][1]
            x1 = self.trajectory.points[i + 1][0]
            y1 = self.trajectory.points[i + 1][1]

            #ax + by = c
            a = y1 - y0
            b = x1 - x0
            c = x0*y1 - x1*y0
            #@Fritz's that calcualte the intersection of circle and line, given ax + by = c and circle at rx, ry with radius r
            
            intersectx = 0
            intersecty = 0

            if(intersectx >= x0 and intersectx <= x1):
                found = True
                break
        
        if(found):
            return (intersectx, intersecty)
        else:
            raise Exception("no intersection found :((")

    def get_curvature(self, goalx, goaly):
        return 2 * goalx / self.lookahead**2
    
    def calculate_lookahead(self):
        pass

    def calculate_lookaheadpoint(self):
        pass

    def drive_command(self):
        AckermannDrive = AckermannDriveStamped()
        AckermannDrive.drive.steering_angle = -np.atan2(2 * self.lookahead * np.sin(nu) / self.lookahead)
        #generalized sttering law by having a point ahead lecture slides
        AckermannDrive.drive.steering_angle = -np.atan2(2 * self.lookahead * np.sin(nu) / self.lookahead)


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
