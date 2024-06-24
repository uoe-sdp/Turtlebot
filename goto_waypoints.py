#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from time import time,sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class waypoint():

	def __init__(self):
		self.pose = None
		self.orien = None
		self.odom_timestamp = None
		self.forward_speed = 1
		self.turning_speed = 1
		rospy.Subscriber('scan',LaserScan,self.laser_scan_callback)
		rospy.Subscriber('odom',Odometry,self.odom_callback)

	def laser_scan_callback(self,data):
		pass

	def odom_callback(self,data):
		self.pose = data.twist.twist
		self.odom_timestamp = data.header.seq

	def move_motor(self,lin,ang):
		pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
		mc = Twist()
		mc.linear.x = lin
		mc.angular.z = ang
		pub.publish(mc)
	def linear_distance(self,distance):
		# Move forward a given distance in cm
		curr_pos_x = 0
		at_waypoint = 0
		lin_threshold = 0.1 #how accurate you want to be to your goal
		old_ts = self.odom_timestamp
		while at_waypoint == 0 and not rospy.is_shutdown():
			if self.odom_timestamp > old_ts:
				curr_pos_x = curr_pos_x + self.pose.linear.x
				old_ts = self.odom_timestamp
				dist_to_goal = curr_pos_x - distance
				print dist_to_goal
				if -lin_threshold < dist_to_goal <lin_threshold:
					self.move_motor(0,0)
					at_waypoint = 1
					return 1
				else:
					if dist_to_goal < 0:
						self.move_motor(self.forward_speed,0)
					if dist_to_goal > 0:
						self.move_motor(-self.forward_speed,0)
		else: 
			self.move_motor(0,0)
	def turn_angle(self,angle_deg):
		# Turn for a given angle (in degrees)
		at_waypoint = 0
		ang_threshold = 5
		curr_heading = 0
		old_ts = self.odom_timestamp
		while at_waypoint == 0 and not rospy.is_shutdown(): 
			try:
				if self.odom_timestamp>old_ts:
					curr_heading += np.rad2deg(self.pose.angular.z)
					old_ts = self.odom_timestamp
					dist_to_goal =  curr_heading - angle_deg
					if -ang_threshold < dist_to_goal < ang_threshold:
						self.move_motor(0,0)
						at_waypoint = 1
						return 1
					else:
						print(dist_to_goal)
						if dist_to_goal < 0:
							self.move_motor(0,self.turning_speed)
						if dist_to_goal > 0:
							self.move_motor(0,-self.turning_speed)
			except:
				self.move_motor(0,0)


if __name__ == '__main__':
	rospy.init_node('example_script',anonymous=True)
	start_time = time()
	duration = 5
	ctrl = waypoint()
	ctrl.forward_speed = 1
	ctrl.turning_speed = 1
        sleep(0.5)

	try:
		ctrl.linear_distance(15)
		ctrl.turn_angle(30)
	except rospy.ROSInterruptException:
		pass
	ctrl.move_motor(0,0)
