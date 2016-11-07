#!/usr/bin/env python

"""
Description: 
    Driver for moving the car (here default car model is Turtlebot)
    1) Subscribe to steer_control and drive_control, get commands
    2) translate control into a Twist object for driving the vehile
    3) drive the vehicle
    
Sample usage:
    from move_car import MOVE_CAR
    mc = MOVE_CAR()
    mc.run()

Author:
    Tian Zhou (zhou338@purdue.edu)
    Maria E. Cabrera (cabrerm@purdue.edu)
    Teerachart Soratana (tsoratan@purdue.edu)

Date: 
    Nov, 7, 2016

License: 
    GNU General Public License v3
"""

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

class MOVE_CAR:
    def __init__(self):
        # init node to drive car
        rospy.init_node('move_car', anonymous=True)
        
        # get steer_control
        rospy.Subscriber("steer_control", Float32, self.steer_control_callback)
        
        # get drive_control
        rospy.Subscriber("drive_control", Float32, self.drive_control_callback) 
        
        # publisher for moving turtlebot
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        
        # init twist
        self.twist = Twist()
        
    def steer_control_callback(self, msg):
        # callback for steer_control, rotating velocity based on this
        if not math.isnan(msg.data):
            self.twist.angular.z = -msg.data

    def drive_control_callback(self, msg):
        # callback for steer_control, determine speed
        if not math.isnan(msg.data):
            self.twist.linear.x = 0.1 # move vertically

    def run(self):
        rate = rospy.Rate(10) # publish at 10 Hz
        while not rospy.is_shutdown():
          self.cmd_vel_pub.publish(self.twist)
          rate.sleep()
  
def main():
    mc = MOVE_CAR()
    mc.run()
    
if __name__ == '__main__':
    main()
