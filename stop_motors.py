#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from time import time

def move_motor(lin,ang):
	pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
	mc = Twist()
	mc.linear.x = lin
	mc.angular.z = ang
	pub.publish(mc)

if __name__ == '__main__':
	rospy.init_node('stop_motors',anonymous=True)
	start_time = time()
	duration = 2
	while time()<start_time+duration:
		try:
			move_motor(0,0)
		except rospy.ROSInterruptException:
			pass
	else:
		move_motor(0,0)
