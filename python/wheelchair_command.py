#!/usr/bin/env python
import time

import classification_realtime as cr
import rospy # pylint: disable=import-error
from geometry_msgs.msg import Twist # pylint: disable=import-error

# ------------------------------------------------------------------
# conditional operational values
# ------------------------------------------------------------------

normal_linear = 20
normal_angular = 100
topic = 'cmd_vel'

demo = True

if demo:
	topic = 'turtle1/cmd_vel'
	normal_linear = 0.5
	normal_angular = 1.0

# ------------------------------------------------------------------
# create rospy interfaces
# ------------------------------------------------------------------

rospy.init_node('Wheelcom')
pub = rospy.Publisher(topic, Twist, queue_size=1)

# ------------------------------------------------------------------
# define operational functions
# ------------------------------------------------------------------


def forward():
	twist = Twist()
	twist.linear.x = normal_linear
	pub.publish(twist)


def backward():
	twist = Twist()
	twist.linear.x = -normal_linear / 2
	pub.publish(twist)


def right():
	twist = Twist()
	twist.angular.z = -normal_angular
	pub.publish(twist)


def left():
	twist = Twist()
	twist.angular.z = normal_angular
	pub.publish(twist)


def forward_left():
	twist = Twist()
	twist.linear.x = normal_linear
	twist.angular.z = normal_angular
	pub.publish(twist)


def forward_right():
	twist = Twist()
	twist.linear.x = normal_linear
	twist.angular.z = -normal_angular
	pub.publish(twist)


def backward_left():
	twist = Twist()
	twist.linear.x = -normal_linear/2
	twist.angular.z = normal_angular
	pub.publish(twist)


def backward_right():
	twist = Twist()
	twist.linear.x = -normal_linear/2
	twist.angular.z = -normal_angular
	pub.publish(twist)


def stop():
	twist = Twist()
	pub.publish(twist)


def decelerate():
	twist = Twist()
	for i in range(3):
		twist.linear.x = 2-i
		pub.publish(twist)
		time.sleep(1)
	

# ------------------------------------------------------------------
# functional processing loop
# ------------------------------------------------------------------


def runROS():
	lastCommand = ''

	while True:
		cmd = cr.getCurGes()
		if cmd != lastCommand:
			lastCommand = cmd
			print cmd
			# if cmd != '': print(cmd)

			if cmd == 'stop':
				stop()
			elif cmd == 'forward':
				forward()
			elif cmd == 'backward':
				backward()
			elif cmd == 'left':
				left()
			elif cmd == 'right':
				right()
			elif cmd == 'forward-left':
				forward_left()
			elif cmd == 'forward-right':
				forward_right()
			elif cmd == 'backward-left':
				backward_left()
			elif cmd == 'backward-right':
				backward_right
			elif cmd == 'Imprecise data. Ignoring.':
				decelerate()
			else:
				print "Unimplemented command:", cmd
				if not demo: time.sleep(5)
				continue

import threading
print "Starting ROS..."
threading.Thread(target=runROS).start()
print "Starting classifier..."
threading.Thread(target=cr.classifyRealtime).start()