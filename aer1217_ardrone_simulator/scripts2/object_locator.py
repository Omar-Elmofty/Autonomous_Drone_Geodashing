#!/usr/bin/env python2


"""ROS Node for performing """

from __future__ import division, print_function, absolute_import


import rospy
import numpy as np
import cv2
import copy
import scipy.misc
import PIL
from blob_detect import BlobDetector
from coord_transf import CoordinateTransform

from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_matrix


class ObjectLocator(object):
	"""Class for locating obstacles and landmarks
	"""
	def __init__(self):
		#Define Subscribers
		self.image_sub = rospy.Subscriber('/ardrone/bottom/image_raw', 
											Image, self.image_callback)
		self.sub_vicon = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre', 
		                   TransformStamped,
		                   self.update_quadrotor_state)

		#Define global variables
		self.image = Image
		self.bridge = CvBridge()
		self.quad_state = TransformStamped()
		self.T_body2vicon = np.identity(4)

		rospy.wait_for_message('/ardrone/bottom/image_raw', Image)
		
		#Define blob detector for detecting obstacles and landmarks
		self.blob_detect = BlobDetector()
		#define coordinate transformation object for georeferencing 
		self.coord_transf = CoordinateTransform()

		#Define 2D grid for storing obstacles and landmarks pixels
		self.grid = np.zeros((1000,1000))

		#time flags for detecting when the rosbag is done publishing
		self.current_time =  rospy.Time.now().to_sec() 
		self.last_msg_time =  rospy.Time.now().to_sec() 

	def image_callback(self,msg):
		"""Callback for image topic
		"""

		#store the current pose which will be attached to the current 
		#camera frame
		current_quad_state = copy.deepcopy(self.quad_state)
		self.image = msg
		self.T_body2vicon[0,3] = current_quad_state.transform.translation.x
		self.T_body2vicon[1,3] = current_quad_state.transform.translation.y
		self.T_body2vicon[2,3] = current_quad_state.transform.translation.z

		q = [ current_quad_state.transform.rotation.x,
		current_quad_state.transform.rotation.y,
		current_quad_state.transform.rotation.z,
		current_quad_state.transform.rotation.w]

		R = quaternion_matrix(q)

		self.T_body2vicon[0:3,0:3] = R[0:3,0:3]

		#Time flag
		self.last_msg_time =  rospy.Time.now().to_sec() 


	def update_quadrotor_state(self, msg):
		"""Callback for vicon topic
		"""
		self.quad_state = msg


	def detect_pixels(self, image):
		"""Function that detects pixels of landmarks and obstacles by 
		thresholding the background of the image
		Args:
			image: input image from drone
		Returns:
			coords: coord's of obstacle or landmark pixels
		"""

		#convert image to hsv and threshold background
		img_lower_thres = np.array([0,5,0], dtype ='uint8')
		img_upper_thres = np.array([255,255,255], dtype ='uint8')
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)		
		mask = cv2.inRange(hsv, img_lower_thres, img_upper_thres)

		#locate indices of obstacles and landmarks
		indicies = np.where(mask == [255])
		coords = zip(indicies[1], indicies[0])

		return coords

	def populate_2D_grid(self):
		""" Function that populates the 2D grid with all the obstacle
		and landmark pixels
		"""

		#Run ROS loop as long as project.bag is publishing
 		while not rospy.is_shutdown():

 			#Exit rosloop if project.bag is done publishing
			self.current_time =  rospy.Time.now().to_sec() 
			if (self.current_time - self.last_msg_time) > 2:
				break

			#Store the current pose and image
			T_b2vic =  copy.deepcopy(self.T_body2vicon)
			img =  copy.deepcopy(self.image)

			#convert image to opencv format
			cv2_img = self.bridge.imgmsg_to_cv2(img, "bgr8")

			#detect obstacle pixels
			coords = self.detect_pixels(cv2_img)

			#populate the 2D grid
			for coord in coords:
				u = coord[0]
				v = coord[1]
				#transform the obstacle pixels to vicon frame
				p = self.coord_transf.cam2vicon_transform(u, v, T_b2vic)
				#add point to grid
				x_idx = int(np.round(p[0,0]*100))
				y_idx = int(np.round(p[1,0]*100))
				self.grid[y_idx, x_idx] = 255
  	
  	def detect_landmarks_obstacles(self):
  		"""Function that detects the obstacles and landmarks locations
  		""" 
  		#Convert grid format
		self.grid = (self.grid).astype('uint8')
		#detect obstacles centroids and radii
		obstacles = self.blob_detect.detect_obstacles(self.grid)
		#detect landmark centroids
		landmarks = self.blob_detect.detect_landmarks(self.grid)

		#print results
		print('Obstacles: ')
		for obs in obstacles:
			print('x=',str(obs[0]/100.0),', y=',str(obs[1]/100.0), ', r=', str(obs[2]/100.0))

		print('Landmarks: ')
		for l in landmarks:
			print('x=',str(l[0]/100.0),', y=',str(l[1]/100.0))

		#visualize the grid
		cv2.imshow('out', self.grid)
		cv2.imwrite('grid.png', self.grid)
		cv2.waitKey(0)  


if __name__ == '__main__':
	rospy.init_node('ObjectLocator')
	g = ObjectLocator()
	g.populate_2D_grid()
	g.detect_landmarks_obstacles()
