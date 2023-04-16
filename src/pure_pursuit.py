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

    def get_coordinate_arrray(self, poses):
        #Return an array of line segements where each segement represented as a 2x2 array of start and end coordinates
        return np.array([[pose.position.x, pose.position.y] for pose in poses])
    
    def get_minimum_distance(v, w, p):
        #Return minimum distance between line segment vw and point p
        l2 = np.linalg.norm(v-w)**2  # |w-v|^2
        if (l2 == 0.0):
             return np.linalg.norm(p-v)**2 ;  # v == w case
        #Consider the line extending the segment, parameterized as v + t (w - v).
        # We find projection of point p onto the line. 
        #It falls where t = [(p-v) . (w-v)] / |w-v|^2
        #We clamp t from [0,1] to handle points outside the segment vw.
        t = max(0, min(1, np.dot(p - v, w - v) / l2))
        projection = v + t * (w - v);  # Projection falls on the segment
        return np.linalg.norm(p - projection)

    def find_closest_segment_index(self, co_ordinate_array, current_location):
        get_closest_points = np.vectorize(self.get_minimum_distance, signature='(n)->()')
        closest_points = get_closest_points(co_ordinate_array[:-1], co_ordinate_array[1:], current_location)
        return np.argmin(closest_points)

    def find_circle_intersection(self, index):
        radius = self.lookahead
        rx = self.x
        ry = self.y
        Q = np.array([rx,ry])
        found = False
        #assume the index is the index of the line segment that the closest point lines on

        for i in range(index, self.n - 1):
            x0 = self.trajectory.points[i][0]
            y0 = self.trajectory.points[i][1]
            x1 = self.trajectory.points[i + 1][0]
            y1 = self.trajectory.points[i + 1][1]

            #ax + by = c
            a = y1 - y0
            b = x1 - x0
            c = x0*y1 - x1*y0
            #@Fritz's that calcualte the intersection of circle and line, given ax + by = c and circle at rx, ry with radius r
            # convert line to slope intercept form, and then vectorize it
            mb = np.array([-a/b,c/b])
            p1 = np.array([x0,y0])
            p2 = np.array([x1,y1])
            v = (p2-p1)/np.linalg.norm(p2-p1) #Unit Norm
            # q_ = variables for quadratic equation for finding points on circ
            qa = v.dot(v)
            qb = 2*v.dot(p1-Q)
            qc = p1.dot(p1) + Q.dot(Q) - 2*p1.dot(Q) - radius**2

            disc = b**2-4*a*c # If negative, line doesn't intersect circle
            t = np.array([(-b+np.sqrt(disc))/(2*a),(-b-np.sqrt(disc))/(2*a)])
            # if either of the t's are outside of (0,1), line segment doesn't touch circle
            intersectpoints = np.array([p1+t[0]*v,p1+t[1]*v]) # Np array of both intersect points
            intersectx = intersectpoints[0][0] # arbitrarily picks first intersect point, may need to be changed
            intersecty = intersectpoints[0][1]

            if(intersectx >= x0 and intersectx <= x1):
                found = True
                break
        if(found):
            return (intersectx, intersecty)
        else:
            raise Exception("no intersection found :((")

    def get_curvature(self, goalx, goaly):
        return 2 * goalx / self.lookahead**2

    def drive_command(self):
        AckermannDrive = AckermannDriveStamped()
        AckermannDrive.drive.steering_angle = -np.atan2(2 * self.lookahead * np.sin(nu) / self.lookahead)
        #generalized sttering law by having a point ahead lecture slides
        AckermannDrive.drive.steering_angle = -np.atan2(self.wheelbase_length * np.sin(nu) / self.lookahead)


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
