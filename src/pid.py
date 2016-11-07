#!/usr/bin/env python

"""
Description: 
	A discrete PID controller
	window size is set to 2 (only current error and previous error)

Sample usage:
	from pid import PID
	import numpy as np

	pid = PID()
	pid.set_PID(kp = 1.0, ki = 0, kd = 1.0)
	while 1:
		pid.input_error(np.random.rand())
		control = pid.output_control() 
		print "Control: %.3f" % control

Author:
	Tian Zhou (zhou338@purdue.edu)

Date: 
	Nov, 4, 2016

License: 
	GNU General Public License v3
"""
import numpy as np

class PID:
	def __init__(self):
		# default PID coefficients
		self.kp = 1.0
		self.ki = 1.0
		self.kd = 1.0

		# default value
		self.e_old = 0.0
		self.e = 0.0

	def set_PID(self, kp, ki, kd):
		# set PID controller coefficient
		self.kp = kp
		self.ki = ki
		self.kd = kd
		print "Set kp: %.3f, ki: %.3f, kd: %.3f" % (kp, ki, kd)

	def input_error(self, e):
		# update the current error and old error
		self.e_old = self.e
		self.e = e

	def output_control(self):
		# Proportional
		p = self.kp * self.e

		# Integration
		i = self.ki * (self.e_old + self.e)

		# Differentiation
		d = self.kd * (self.e - self.e_old)

		# sum
		control = p + i + d

		return control


def main():
	pid = PID()
	pid.set_PID(kp = 1.0, ki = 0, kd = 1.0)
	while 1:
		pid.input_error(np.random.rand())
		control = pid.output_control() 
		print "Control: %.3f" % control

if __name__ == '__main__':
	main()
