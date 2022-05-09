#!/usr/bin/env python

## This is task two solution of the PARC Engineers League. This solution was
## developed by Kognitive Robotics team from Nigeria.

## The camera scene from the robot was passed through a colour filter to get 
## the HSV (Hue Saturation Value) of the traffic sign. This information was then 
## used to detect the colour, shape, and state of the traffic sign. The robot 
## remains in its initial position when the red sign is active and upon a go-state 
## (or the green sign detection) it keeps moving and checking whether it has 
## reached the goal location using the pose information from the odometry, and 
## stops immediately it gets to the goal point.

## Check "hsv_colour_filter.py" code to see how the traffic sign colours were gotten.

## Author: Sikiru Salau
## Contributor: George Okoroafor


import rospy
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry


# x_pos stores the x coordinate of the robot pose.
x_pos = 0

# Holds value to be published to cmd_vel to move the robot.
speed_cmd = Twist()

# Set to False if you do not want to see robot camera display.
show_frame = True

# Variable to check the state of the traffic sign.
# state is FALSE when RED sign is active and TRUE when GREEN is active.
state = False


# A class that handles subscribing, publishing, traffic light detection,
# traffic state recognition and goal location check.
class TrafficSignNav(object):

	# Initialize variables to hold subscriber and publisher values.
	def __init__(self):

		# Subscribe to camera topic and store value in image_sub
		self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.cameraCb)

		# Subscribe to odometry topic and store value in odom_sub
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCb)

		# Publish cmd_vel_pub to command velocity (cmd_vel) topic
		self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

		# Object that will be used to translate ROS message type to OpenCV type
		self.bridge_object = CvBridge()

	# Calls back the x coordinate of the robot pose
	def odomCb(self, pos):
		global x_pos
		x_pos = pos.pose.pose.position.x

	# Process camera information, detect and recognize traffic state,
	# navigate the robot based on processed information
	def cameraCb(self, data):
		global x_pos
		global state
		global speed_cmd
		global show_frame

		# Error handling
		try:
			# Select bgr8. It is the OpenCV encoding by default.
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError as e:
			print(e)

		# Convert the image in cv_image from RGB to HSV.
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

		# Set range for green colour.
		lower_green = np.array([26, 177, 226])
		upper_green = np.array([179, 255, 247])

		# Set range for red colour.
		lower_red = np.array([0, 134, 158])
		upper_red = np.array([0, 255, 255])

		# Threshold the HSV image to get only green colors.
		green_mask = cv2.inRange(hsv, lower_green, upper_green)
		# Uncomment the line below to get only green with black background
		# green = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)

		# Threshold the HSV image to get only red colors.
		red_mask = cv2.inRange(hsv, lower_red, upper_red)
		# Uncomment the line below to get only red with black background
		# red = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)

		# Hold information about the features of the red object
		cnts,hei = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]

		# Identify the shape of red object.
		for c in cnts:
			area = cv2.contourArea(c)
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.02*peri, True)

			# Check if the traffic sign is in a STOP state.
			if state == False:
				# If the red object is a traffic sign in its stop state.
				if len(approx) == 8:
					if area>200:
						# Display a white rectangular box around the red traffic sign
						# with a text indicating STOP.
						x,y,w,h = cv2.boundingRect(c)
						cv2.rectangle(cv_image, (x,y), (x+w, y+h), (255,255,255),2)
						cv2.putText(cv_image, "Stop", (x, y+h+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)

						# Ensure the robot is not moving.
						speed_cmd.linear.x = 0.0
						self.cmd_vel_pub.publish(speed_cmd)
					else:
						# Do nothing if you can't see the traffic sign.
						pass
				else:
					# You dare not do anything if the red object is not a traffic sign.
					pass
			else:
				# Red, stay idle if green state is active.
				pass

		# Hold information about the features of the green object
		cnts,hei = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]

		# Identify the shape of green object.
		for c in cnts:
			area = cv2.contourArea(c)
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.02*peri, True)

			# If the green object is a traffic sign in its go state
			if len(approx)==8:
				# Green is active
				state = True
				if area>20:
					# Display a white rectangular box around the green traffic sign
					# with a text indicating GO.
					x,y,w,h = cv2.boundingRect(c)
					cv2.rectangle(cv_image, (x,y), (x+w, y+h), (255,255,255),2)
					cv2.putText(cv_image, "Go", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255,255,255), 2)

					# An idle robot is the devil's workshop. Haha! Please move forward.
					speed_cmd.linear.x = 1.6
					self.cmd_vel_pub.publish(speed_cmd)
				else:
					pass
			else:
				pass

		# Make sure the robot stops at the goal location.
		if x_pos > -2.1:
			speed_cmd.linear.x = 0.0
			self.cmd_vel_pub.publish(speed_cmd)

		# Me: Seeing is believing.
		# Robot: Just have faith in me
		# Me: Gosh! LMAO! Com'on, show me what's going on.
		if show_frame == True:
			cv2.imshow("Traffic Sign Navigation", cv_image)

			# Uncomment the lines below if you want to see the green, red, or hsv display.
			# cv2.imshow("Green Mask", green)
			# cv2.imshow("Red Mask", red)
			# cv2.imshow("HSV", hsv)
			
		else:
			# I'm okay with the display from Gazebo and RViz only.
			pass

		cv2.waitKey(30)

def main():
	# Initialize the node.
	rospy.init_node('traffic_signal_navigation', anonymous=True)

	# Run the class, TrafficSignNav.
	TrafficSignNav()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

	# Does this sound like a destroyer? It should've been called closeAllWindows. I guess that's what it does.
	cv2.destroyAllWindows()

if __name__ == '__main__':
	# Make Isaac Asimov proud of his robot laws.
	main()
	