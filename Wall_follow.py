#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 1.1
kd = 0.0
ki = 0.0
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 1.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) 
        self.drive_pub = rospy.Publisher(drive_topic , AckermannDriveStamped, queue_size=10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        a = data.ranges[int(0 + (0.5*np.pi)/data.angle_increment)]
        b = data.ranges[int(0 + (0.5*np.pi + angle/180)/data.angle_increment)] 
        alfa = math.atan( (a*math.cos(angle) - b) / a*math.sin(angle) )
        Dt = b * math.cos(alfa)
        return Dt + 0.4*math.sin(alfa) 
        

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        #TODO: Use kp, ki & kd to implement a PID controller for 
        
        integral += error
        deriver = error - prev_error
        angle = kp*error + kd*deriver + ki*integral 
       
        if (angle>=0) and (angle<10):
        	velocity = 1.5
        if (angle>=10) and (angle<20):
        	velocity = 1
        if (angle>=20):
        	velocity = 0.5
       
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        

    def followRight(self, data, rightDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        return rightDist - self.getRange(data,50)

    def lidar_callback(self, data):
        error = self.followRight(data, DESIRED_DISTANCE_RIGHT) #TODO: replace with error returned by followRight
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)

