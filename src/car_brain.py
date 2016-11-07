#!/usr/bin/env python

"""
Description: 
    The car controller, do the following things
    1) set 1 PID controller for steer_control and drive_control each
    2) get error from steer_e and drive_e
    3) calculate control for steer_control and drive_control
    4) publish the control to steer_control and drive_control
    
Sample usage:
    from car_brain import CAR_BRAIN
    car = CAR()
    car.set_steer_PID(kp=1, ki=0, kd=1)
    car.set_drive_PID(kp=1, ki=0, kd=1)
    rospy.spin()

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
from sensor_msgs.msg import LaserScan
from pid import PID
import numpy as np
from std_msgs.msg import Float32

class CAR_BRAIN:
    def __init__(self):
        # debug mode
        self.debug = False

        # init node
        rospy.init_node('car_brain', anonymous=True)

        # init PID for steer
        self.steer_pid = PID()
        
        # init PID for drive 
        self.drive_pid = PID()

        # get steer_e
        rospy.Subscriber("steer_e", Float32, self.steer_e_callback)
        self.steer_pub = rospy.Publisher('steer_control', Float32, queue_size=10)
        # self.rate = rospy.Rate(10) # publish at 10 Hz

        # get drive_e
        # drive_e should be published by a MAP with WAYPOINTS
        # just use steer_e now as place holder!!!!
        # !!!!!!!!!!!!! UPDATE!!!!!!!!!!!!!!!!!!!!!
        rospy.Subscriber("steer_e", Float32, self.drive_e_callback) 
        self.drive_pub = rospy.Publisher('drive_control', Float32, queue_size=10)

    def steer_e_callback(self, msg):
        # callback for steer_e, Input error, calculate PID control output
        # publish control output to topic steer_control (type Float32)
        self.steer_pid.input_error(msg.data)
        steer_control = self.steer_pid.output_control() 
        if self.debug:
            print "steer_control: %.3f" % steer_control

        # publish to a topic called "steer_control"
        self.steer_pub.publish(steer_control)
        
    def set_steer_PID(self, kp, ki, kd):
        # set steer PID control coefficients
        print "Set steer PID: "
        self.steer_pid.set_PID(kp = kp, ki = ki, kd = kd)

    def drive_e_callback(self, msg):
        # callback for steer_e, Input error, calculate PID control output
        # publish control output to topic drive_control (type Float32)
        self.drive_pid.input_error(msg.data)
        drive_control = self.drive_pid.output_control() 
        if self.debug:
            print "drive_control: %.3f" % drive_control

        # publish to a topic called "drive_control"
        self.drive_pub.publish(drive_control)
        
    def set_drive_PID(self, kp, ki, kd):
        # set drive PID control coefficients
        print "Set drive PID: "
        self.drive_pid.set_PID(kp = kp, ki = ki, kd = kd)   

def main():
    car = CAR_BRAIN()
    car.set_steer_PID(kp=1.0, ki=0.0, kd=0.0)
    car.set_drive_PID(kp=1.0, ki=0.0, kd=0.0)
    rospy.spin()

if __name__ == '__main__':
    main()