#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, ColorRGBA
from sensor_msgs.msg import Joy

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic","/pf/pose/odom")
        self.lookahead        = 1.5 #filled in for testing purposes, please update, larger is more smooth and smaller is more oscillations
        self.speed            = -0.4 #filled in for testing purposes, please update
        self.wheelbase_length = 0.32 #flilled in for testing purposes, please update
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        
        self.in_sim = True
        if not self.in_sim:
            rospy.logerr("testing on car!")
            self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
            self.drive_pub2 = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
            self.teleop_subscriber = rospy.Subscriber("/vesc/joy", Joy, self.tcb, queue_size=1)    
        else:
            rospy.logerr("testing in sim!")
            self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        
        self.cterr_pub = rospy.Publisher("/crosstrackerror", Float64,queue_size=1)
        self.goal_point_pub = rospy.Publisher('/pure_pursuit_goal', Marker, queue_size = 1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.current_location = np.array([0,0])
        self.x = self.current_location[0]
        self.y = self.current_location[1]
        self.theta = 0
        self.brake = False # Boolean condition to determine whether to stop
        self.thresh = 1.0 # distance from final path point at which car will stop
    
        self.pressed = False	
        rospy.logerr("backwards pure pursuit!")

    def tcb(self, msg):
        buttons = msg.buttons
        if(buttons[5] == 1):
            self.pressed = True
        else:
            self.pressed = False

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        #print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        rospy.logerr("traj callback!")

    def odom_callback(self, msg):
        curposition = msg.pose.pose.position
        curorientation = msg.pose.pose.orientation
        self.current_location = np.array([curposition.x,curposition.y])
        self.x = curposition.x
        self.y = curposition.y
        poses = self.trajectory.points
        if(poses == []):
            return
        
        self.n = len(poses)
        coordarr = self.get_coordinate_array2(poses)
        index = self.find_closest_segment_index(coordarr,self.current_location)
        goalpos = self.find_circle_intersection(index)

        if (np.linalg.norm(self.current_location - np.array(poses[0])) < self.thresh): # Car stop condition
            self.brake = True
        else:
            self.brake = False

        if(goalpos != (0, 0)):
            goalpos = self.transformtocarframe(goalpos, curorientation, curposition)
            goalpos = (goalpos[0], goalpos[1])
            self.drive_command(goalpos[0],goalpos[1])


    def get_coordinate_array(self, poses):
        #Return an array of line segements where each segement represented as a 2x2 array of start and end coordinates
        return np.array([[pose.position.x, pose.position.y] for pose in poses])
    
    def get_coordinate_array2(self, poses):
        #Return an array of line segements where each segement represented as a 2x2 array of start and end coordinates
        return np.array([[pose[0], pose[1]] for pose in poses])
    
    def get_minimum_distance(self, v, w, p):
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
        get_closest_points = np.vectorize(self.get_minimum_distance, signature='(n),(n),(n)->()')
        closest_points = get_closest_points(co_ordinate_array[:-1], co_ordinate_array[1:], current_location)
        return np.argmin(closest_points)

    def find_circle_intersection(self, index):
        if(self.speed > 0):
            if self.speed == 4:
                radius = 3.3425
                #3.340 was good but slightly drifting near the column
                #3.3425 went out near the column
                #3.3437
                #3.345 was oversmooth
            if self.speed == 1:
                radius = 1.68
                #0.845 was too smooth
                #0.835 worked pretty well but drifted on Penny's computer
            if self.speed == 2:
                radius = 1.67
                #1.67 had a lot of odometry drifting
            if self.speed == 3:
                radius = 2.50
            else:
                radius = (3.3425/4.0 * self.speed)
        if self.speed < 0:
            radius = abs(3.3425/4.0 * self.speed) + 0.32

        rx = self.x
        ry = self.y
        Q = np.array([rx,ry])
        found = False
        #assume the index is the index of the line segment that the closest point lines on
        #Compute cross track error to closest line segment
        def cte(index): # distance from current location to closest line segment
            x1 = self.trajectory.points[index][0]
            y1 = self.trajectory.points[index][1]
            x2 = self.trajectory.points[index+1][0]
            y2 = self.trajectory.points[index+1][1]
            px = x2 - x1
            py = y2 - y1
            norm = px*px + py*py
            u = ((self.current_location[0]-x1)*px + (self.current_location[1]-y1)*py)/float(norm)
            if u>1:
                u=1
            elif u<0:
                u=0
            x = x1 + u*px
            y = y1 + u*px

            dx = x - self.current_location[0]
            dy = y - self.current_location[1]
            dist = (dx*dx + dy*dy)**.5
            return dist
        cterr = Float64()
        cterr.data = cte(index)
        self.cterr_pub.publish(cterr)

        #rospy.logerr(self.trajectory.points)
        for i in range(index, -1, -1):
            x0 = self.trajectory.points[i][0]
            y0 = self.trajectory.points[i][1]
            x1 = self.trajectory.points[i + 1][0]
            y1 = self.trajectory.points[i + 1][1]

            p1 = np.array([x0,y0])
            p2 = np.array([x1,y1])
            v = (p2-p1)

            qa = np.dot(v, v)
            qb = 2.0 * np.dot(v, p1 - Q)
            qc = np.dot(p1, p1) + np.dot(Q, Q) - 2.0 * np.dot(p1, Q) - radius**2
            # print("seg: ({},{})->({},{})".format(x0,y0,x1,y1))

            disc = qb**2.0-4.0*qa*qc # If negative, line doesn't intersect circle
            # print("disc: {}".format(disc))
            if(disc < 0):
                continue
            t = np.array([(-1.0 * qb+np.sqrt(disc))/(2.0*qa),(-1.0 * qb-np.sqrt(disc))/(2.0*qa)])
            # print(t)
            # take abs value of t values to ensure direction along line segment is correct
            # if either of the t's are outside of (0,1), line segment doesn't touch circle
            solutionpoints = []
            if(t[0] < 1 and t[0] > 0):
              p = p1 + t[0] * v
              solutionpoints.append((p[0], p[1]))
              found = True
            if(t[1] < 1 and t[1] > 0):
              p = p1 + t[1] * v
              solutionpoints.append((p[0], p[1]))
              found = True
            if(found):
                answer = solutionpoints[0]
                p = Pose()
                point = Point()
                m = Marker()
                m.header.frame_id = "/map"
                c = ColorRGBA()
                c.r = 255
                c.g = 0
                c.b = 0
                m.type = 2
                m.color = c
                point.x = answer[0]
                point.y = answer[1]
                point.z = 0
                p.position = point
                m.pose = p
                self.goal_point_pub.publish(m)
                return answer
        return (0, 0)

    def get_curvature(self, goalx, goaly):
        return 2 * goalx / self.lookahead**2
    
    def get_rotation_matrix(self, theta):
        return np.array([[np.cos(theta), -1 * np.sin(theta), 0, 0], [np.sin(theta), np.cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    def transformtocarframe(self, goalpos, orientation, position):
        roll, pitch, theta = euler_from_quaternion([orientation.x, orientation.x, orientation.z, orientation.w])
        rotation_matrix = self.get_rotation_matrix(theta)
        trans_matrix = np.array([[1, 0, 0, position.x], [0, 1, 0, position.y], [0, 0, 1, position.z], [0, 0, 0, 1]])
        
        combined_trans = np.linalg.inv( np.matmul(trans_matrix, rotation_matrix))
        # rospy.loginfo(combined_trans)

        goal = np.array([[goalpos[0]], [goalpos[1]], [0], [1]])
        final_goal = np.matmul(combined_trans, goal)
        #rospy.logerr("FINAL GOAL")
        #rospy.logerr(final_goal)
        return final_goal
    
    def drive_command(self, goalx, goaly):
        #rospy.logerr("x: {}, y: {}".format(self.x, self.y))
        #rospy.logerr("goalx: {}, goaly: {}".format(goalx, goaly))
        #ospy.logerr("a")

        eta = np.arctan2(goaly, goalx)
        # R = self.lookahead / (2 * np.sin(eta))
        AckermannDrive = AckermannDriveStamped()
        AckermannDrive.header.stamp = rospy.Time.now()
        AckermannDrive.header.frame_id = "base_link"
        if (self.brake):
            rospy.logerr("STOPPING)")
            AckermannDrive.drive.speed = 0
            AckermannDrive.drive.steering_angle = 0
        else:
            rospy.logerr("sending drive signal")
            AckermannDrive.drive.speed = self.speed
            AckermannDrive.drive.steering_angle = np.arctan2(2 * self.wheelbase_length * np.sin(eta), np.sqrt(goalx**2 + goaly**2))

        #generalized sttering law by having a point ahead lecture slides
        # lfw = 0.05 #randomly defined based on lecture slides
        # AckermannDrive.drive.steering_angle = -1 * np.arctan(self.wheelbase_length * np.sin(eta) / (self.lookahead / 2  + lfw/np.cos(eta)))
        if not self.in_sim:
            if (self.pressed):
                self.drive_pub2.publish(AckermannDrive)
        self.drive_pub.publish(AckermannDrive)


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
