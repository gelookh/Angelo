#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from ackermann_msgs import AckermannDriveStamped 
from std_msgs import Bool 
import numpy as np

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        pub_mess = rospy.Publisher('brake' , AckermannDriveStamped, queue_size=10)
        pub_break = rospy.Publisher('brake bool' , Boolean, queue_size=10)
        rospy.Subscriber('laser', LaserScan, self.scan_callback)
        rospy.Subscriber('odometry', Odometry, self.odom_callback)
        

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.linear.x
        
    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ttc = [] 
        ack = AckermannDriveStamped() 
        break_bool = False
        inc = scan_msg.angle_increment 
        for x in range (len(scan_msg.ranges)):
        	angle = scan_msg.angle_min + inc*x
        	distance = scan_msg.ranges[x]
        	vitesse = self.speed*np.cos(angle) 
        	ttc[x] = distance/v
        TTC = np.min(ttc)
        
        # TODO: publish brake message and publish controller bool
	if (TTC<0,4):
	        ack.speed = 0 
	        pub_mess.publish(ack) 
	        break_bool = True
	        pub_break.publish(break_bool) 
		

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()

