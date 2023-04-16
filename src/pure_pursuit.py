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
        self.lookahead        = # FILL IN #
        self.speed            = # FILL IN #
        self.wheelbase_length = # FILL IN #
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)

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

    def find_circle_intersection(self):
        pass

    def calculate_lookahead(self):
        pass

    def calculate_lookaheadpoint(self):
        pass

    def drive_command(self):
        pass

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
