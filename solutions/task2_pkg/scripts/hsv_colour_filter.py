#!/usr/bin/env python

# This code is used to get HSV of the traffic light colours.

# Launch "roslaunch parc-robot task2.launch" before
# running this node.

# Author: Sikiru Salau


import rospy
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class ColourFilter(object):

	def __init__(self):

		self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.camera_callback)
		self.bridge_object = CvBridge()

	def empty(img):
		pass

	# Create TrackBar Sliders for HSV colour range
	cv2.namedWindow("HSV Filter")
	cv2.resizeWindow("HSV Filter", 600, 300)
	cv2.createTrackbar("hue_min", "HSV Filter",0,179,empty)
	cv2.createTrackbar("hue_max", "HSV Filter",179,179,empty)
	cv2.createTrackbar("sat_min", "HSV Filter",0,255,empty)
	cv2.createTrackbar("sat_max", "HSV Filter",255,255,empty)
	cv2.createTrackbar("val_min", "HSV Filter",0,255,empty)
	cv2.createTrackbar("val_max", "HSV Filter",255,255,empty)

	def camera_callback(self, data):
		try:
			# We select bgr8 because its the OpenCV encoding by default
			cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError as e:
			print(e)

		# Convert from RGB to HSV
		hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		hue_min = cv2.getTrackbarPos("hue_min", "HSV Filter")
		hue_max = cv2.getTrackbarPos("hue_max", "HSV Filter")
		sat_min = cv2.getTrackbarPos("sat_min", "HSV Filter")
		sat_max = cv2.getTrackbarPos("sat_max", "HSV Filter")
		val_min = cv2.getTrackbarPos("val_min", "HSV Filter")
		val_max = cv2.getTrackbarPos("val_max", "HSV Filter")

		lower_colour = np.array([hue_min, sat_min, val_min])
		upper_colour = np.array([hue_max, sat_max, val_max])

		# Threshold the HSV image to get only colours within the lower and upper boundary
		colour_mask = cv2.inRange(hsv, lower_colour, upper_colour)
		colour = cv2.bitwise_and(cv_image, cv_image, mask=colour_mask)

		cv2.imshow("Camera View", colour)
		#cv2.imshow("HSV", hsv)
		#cv2.imshow("CV_Image", cv_image)

		cv2.waitKey(30)

def main():
	rospy.init_node('hsv_colour_filter', anonymous=True)
	ColourFilter()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()

# hmin = 54
# hmax = 179
# smin = 196
# smax = 255
# vmin = 0
# vmax = 255