#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic , LaserScan , self.lidar_callback) 
        self.drive_pub = rospy.Publisher(drive_topic , AckermannDriveStamped , queue_size=10)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.array(ranges)
        proc_ranges[proc_ranges>3] = 3
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        n = len(free_space_ranges)
        start=0
        end=0
        s=0
        e=0
        for i in range (n):
            if (i==n-1) or (free_space_ranges[i]<2.0):
                if (e-s > end-start):
                    end = e
                    start = s
                e=i+1
                s=i+1
            else :
                e=i+1
        return start, end 
    
    def find_best_point(self, start_i, end_i, ranges):
        #tab = ranges[start_i : end_i] 
        #best = np.max(tab)
        #ind = np.where(tab==best)
        return (start_i + end_i) / 2
        
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        prem = int((-(np.pi)/4 - data.angle_min)/data.angle_increment)
        deux = int(((np.pi)/4 - data.angle_min)/data.angle_increment)
        proc_ranges = proc_ranges[prem : deux]
        
        #Find closest point to Lidar 
        closest = np.min(proc_ranges)
        ind = np.where(proc_ranges == closest)
        indice_closest = ind[0][0]

        
        # Eliminate all point inside 'bubble' 
        start = (indice_closest-35)%len(proc_ranges)
        end = (indice_closest+35)%len(proc_ranges)
        proc_ranges[start : end] = 0
         
        
        #Find max length gap 
        start_best_gap , end_best_gap = self.find_max_gap(proc_ranges)
        
        
        #Find the best point in the gap 
        best_point = self.find_best_point(start_best_gap, end_best_gap, proc_ranges) + prem
        
        angle = (data.angle_min + data.angle_increment*best_point)
        
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = 0.95*angle
        drive_msg.drive.speed = 2.85
        self.drive_pub.publish(drive_msg)
        
def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()
    
if __name__ == '__main__':
    main(sys.argv)
