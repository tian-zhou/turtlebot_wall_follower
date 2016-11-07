#!/usr/bin/env python

"""
Description: 
    1) read lidar_scan from topic hokuyo_scan
    2) find distance to the wall based on geometry described in UPenn lecture 6
    3) set the desired distance to right wall, and theta (angle of range ahead)
        and set set_into_future
    4) calculate the error for steering, based on difference of dis_rwall
        and desired_dis_rwall, publish error to topic "steer_e" (type Float32)
    
Sample usage:
    from lidar_read import LIDAR_READ
    LR = LIDAR_READ()
    LR.set_theta(10)
    LR.set_dis_rwall_T(0.5)
    LR.set_step_into_future(0.5)
    LR.run()

Author:
    Tian Zhou (zhou338@purdue.edu)
    Maria E. Cabrera (cabrerm@purdue.edu)
    Teerachart Soratana (tsoratan@purdue.edu)

Date: 
    Nov, 4, 2016

License: 
    GNU General Public License v3
"""


import rospy
import math
from sensor_msgs.msg import LaserScan
from pid import PID
import numpy as np
from std_msgs.msg import Float32

class LIDAR_READ:
    def __init__(self):
        # flag for debug
        self.debug = False

        # common constant
        self.ang2idx = 1.0 / 0.246 #0.352
        self.idx2ang = 1.0 / self.ang2idx
        self.ang2rad = math.pi / 180.0
        self.rad2ang = 1.0 / self.ang2rad

        # default self.theta value
        self.theta = 20 
        self.dis_rwall_T = 0.5
        self.step_into_future = 0.2
        
        # init ros node
        rospy.init_node('lidar_read', anonymous=True)
        
        # init publisher to publish error
        self.pub = rospy.Publisher('steer_e', Float32, queue_size=10)
        self.rate = rospy.Rate(10) # publish at 10 Hz

    def set_theta(self, theta):
        """
        set hyper parameter theta (in degree) to calculate distance to wall
        using geometry from F1 tutorial 6
        """
        if theta < 0 or theta > 72:
            print "Invalid theta value %i, range [0, 72]" % theta
            exit(-1)
        self.theta = theta

    def set_dis_rwall_T(self, dis_rwall_T):
        """
        set the desired distance to the right wall, in our case 0.5 m
        """
        if dis_rwall_T < 0 or dis_rwall_T > 2:
            print "Invalid dis_rwall value %i, range [0, 2]" % dis_rwall_T
            exit(-1)
        self.dis_rwall_T = dis_rwall_T

    def set_step_into_future(self, step_into_future):
        """
        set the step_into_future, it should be dependent to the car speed 
        and how proative you want the system to be
        """
        if step_into_future < 0:
            print "Invalid step_into_future value %i, range (>=0)" % step_into_future
            exit(-1)
        self.step_into_future = step_into_future

    def scan_callback(self, msg):
        """
        msg.ranges has 512 values
        and probably a step size with 0.352 degree, thus the entire range is 180 
        degrees, starting from right to left.
        """

        # The following two value readings are important and need to be changed for 
        # different sensr placements and sensor value directions and ranges
        right = msg.ranges[-1]
        right_ahead = msg.ranges[len(msg.ranges) - int(self.theta * self.ang2idx)]
        
        dis_rwall = self.calc_dis_to_wall(right, right_ahead, self.theta)
        steer_e = dis_rwall - self.dis_rwall_T

        # publish steer error to topic called steer_e
        self.pub.publish(steer_e)
        
        if self.debug:
            print "right: %.3f, right ahead: %.3f, dis_rwall: %.3f" % \
                (right, right_ahead, dis_rwall)

    def calc_dis_to_wall(self, right, right_ahead, theta):
        """
        calcuate the distance of the car to the right wall, according to geometry
        from the UPenn videos.  
        """
        numerator = right_ahead * math.cos(self.ang2rad * theta) - right
        denominator = right_ahead * math.sin(self.ang2rad * theta)
        steer_rad = math.atan(numerator / denominator) # alpha in radian
        dis_to_wall = right * math.cos(steer_rad)
        proj_dis_future = dis_to_wall + self.step_into_future * math.sin(steer_rad)
        return proj_dis_future

    def run(self):
        """
        run the main loop
        """
        rospy.Subscriber("hokuyo_scan", LaserScan, self.scan_callback)
        rospy.spin()

if __name__ == '__main__':
    LR = LIDAR_READ()
    LR.set_theta(10)
    LR.set_dis_rwall_T(0.5)
    LR.set_step_into_future(0)
    LR.run()

