#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import  math
import numpy as np
import rospy
from SS_2434_ArUco_library import *


class image_proc():

	# Initialise everything
	def __init__(self):
		
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=1)
		self.r = rospy.Rate(10)
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw

		rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker
	
	# Callback function of camera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			self.publish_data()
			self.r.sleep()
		except CvBridgeError as e:
			print(e)
			return

	def detect_ArUco(self, img):

		ret_list = [0]
		try:
			aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			parameters = aruco.DetectorParameters_create()
			corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
			ret_list = [ids[0][0], corners[0][0]]

		except:
			pass

		return ret_list

	def mid_and_angle(self, ret_list):
		final_lst = [0, 0, 0]
		try:
			mid_x = 0
			mid_y = 0
			for i in range(4):
				mid_x = mid_x + ret_list[1][i][0]
				mid_y = mid_y + ret_list[1][i][1]
			mid_x = mid_x / 4
			mid_y = mid_y / 4
			edge_x = (ret_list[1][0][0] + ret_list[1][1][0]) / 2
			edge_y = (ret_list[1][0][1] + ret_list[1][1][1]) / 2
			edge_x = edge_x - mid_x
			edge_y = mid_y - edge_y
			dist = ((edge_x) ** 2 + (edge_y) ** 2) ** 0.5
			angle = math.asin((edge_y / dist))
			if angle >= 0 and edge_x < 0:
				angle = math.pi - angle

			elif angle < 0:
				if edge_x >= 0:
					angle = 2 * math.pi + angle
				else:
					angle = math.pi - angle
			angle = ((180 / math.pi) * angle)
			final_lst = [mid_x, mid_y, angle]


		except:
			pass

		return final_lst

	def publish_data(self):
		ret_lst = self.detect_ArUco(self.img)
		self.marker_msg.id = ret_lst[0]
		final_lst = self.mid_and_angle(ret_lst)
		self.marker_msg.x = final_lst[0]
		self.marker_msg.y = final_lst[1]
		self.marker_msg.yaw = final_lst[2]
		self.marker_pub.publish(self.marker_msg)


if __name__ == '__main__':
    image_proc_obj = image_proc()
    rospy.spin()
