#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
import csv
import tf
import numpy as np
import math as m


# TODO: import ROS msg types and libraries

file = open('/home/angelo/catkin_ws/src/f1tenth_simulator/src/scripts/Waypoint_torino.csv')
csv_reader = csv.reader(file)
points = [] 

for row in csv_reader:
    for point in row : 
        points.append(float(point))

points = np.array(points)

file.close()
L=1

def distance(a,b,c,d):

    return np.sqrt((c-a)**2 + (d-b)**2)

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        #TODO: create Ros subscribers and publishers
        
        pos_sub = rospy.Subscriber('/gt_pose' , PoseStamped , self.pose_callback)
        self.drive_pub = rospy.Publisher('/nav' , AckermannDriveStamped , queue_size=10) 
        
        self.i = 0
        
    def pose_callback(self, pose_msg):
        
        # TODO: find the current waypoint to track using methods mentioned in lecture
        
        abscice = float(pose_msg.pose.position.x) 
        ordon = float(pose_msg.pose.position.y)
        
        while distance(abscice,ordon,points[self.i],points[self.i+1])<L:
            self.i = self.i+4
        
        xp = points[self.i]
        yp = points[self.i+1]
        l = distance(abscice,ordon,points[self.i],points[self.i+1])
        
        quat = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, 
                           pose_msg.pose.orientation.w])
                           
        euler = tf.transformations.euler_from_quaternion(quat)
        lacet = euler[2]
        teta = m.atan2(yp - ordon , xp - abscice) 
        
        alpha = teta - lacet 
        
        while alpha > np.pi: 
            alpha = alpha - 2*np.pi
        while alpha <= -np.pi:
            alpha = alpha + 2*np.pi
            
        if alpha > np.pi/2 :
            angle_curv = 0.4188
        elif alpha < - np.pi/2 :
            angle_curv = -0.4188
        else : 
            a = l*m.sin(alpha) 
            angle_curv = (2*a)/(l*l)
            while angle_curv > np.pi : 
                angle_curv = angle_curv - 2*np.pi
            while angle_curv <= - np.pi:
                angle_curv = angle_curv + 2*np.pi
            angle_curv = min(angle_curv,0.4188)
            angle_curv = max(angle_curv,-0.4188)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = 0.8*angle_curv
        drive_msg.drive.speed = 4.2
        self.drive_pub.publish(drive_msg)
        
def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()
