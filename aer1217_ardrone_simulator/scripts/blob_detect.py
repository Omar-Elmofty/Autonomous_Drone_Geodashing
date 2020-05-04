import numpy as np
import cv2

from tf.transformations import quaternion_matrix
import PIL
from PIL import Image as IM
import imutils


class BlobDetector():
	"""Class for locating blobs
	"""
	def __init__(self):

		#Initiate simple blob detector for obstacles
		params = cv2.SimpleBlobDetector_Params()
		params.filterByArea = True
		params.minArea = 10
		params.maxArea = float('inf')
		params.filterByCircularity = True
		params.minCircularity = 0.8
		params.maxCircularity = 1.0
		params.filterByInertia = False
		params.filterByConvexity = False
		params.filterByColor = True
		params.blobColor = 255

		self.detector_obs = cv2.SimpleBlobDetector_create(params)

		#Initiate simple blob detector for landmarks
		params = cv2.SimpleBlobDetector_Params()
		params.filterByArea = True
		params.minArea = 10
		params.maxArea = float('inf')
		params.filterByCircularity = True
		params.minCircularity = 0.0
		params.maxCircularity = 0.8
		params.filterByInertia = False
		params.filterByConvexity = False
		params.filterByColor = True
		params.blobColor = 255

		self.detector_landmarks = cv2.SimpleBlobDetector_create(params)
	
	def detect_obstacles(self, image):
		"""Function that locates the obstacles from input image"""

		keypoints = self.detector_obs.detect(image)
		obstacles = []   
		for i in range(len(keypoints)):
		    obstacles.append([keypoints[i].pt[0], keypoints[i].pt[1], keypoints[i].size/2.0])
	
		return obstacles

	def detect_landmarks(self, image):
		"""Function that locates landmarks from input image"""

		keypoints = self.detector_landmarks.detect(image)

		landmarks = []   
		for i in range(len(keypoints)):
		    landmarks.append([keypoints[i].pt[0], keypoints[i].pt[1]])
	
		return landmarks


# class LandmarkClassifier():
# 	"""Class for classifying landmarks
# 	"""
# 	def __init__(self):
# 		#Load Reference images
# 		img1 = cv2.imread('cn_tower.png')
# 		img2 = cv2.imread('casa_loma.png')
# 		img3 = cv2.imread('nathan_philips_square.png')
# 		img4 = cv2.imread('princes_gates.png')

# 		#extract orb features
# 		landmarks = [img1, img2, img3, img4]
# 		self.orb = cv2.ORB_create(nfeatures=1000)
# 		self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
# 		self.descs = []
# 		for landmark in landmarks:
# 		    kp, des= self.orb.detectAndCompute(landmark,None)
# 		    self.descs.append(des)

# 	def classify_landmarks(self, image):
# 		"""Function that classifies landmarks"""
		
# 		kpq, desq = self.orb.detectAndCompute(image,None)

# 		num_matches = []
# 		for des in self.descs:
# 			try:
# 				matches = self.bf.match(des,desq)
# 				num_matches.append(len(matches))
# 			except:
# 				num_matches.append(0)
# 				print('couldnt find matches')

# 		if np.max(num_matches) > 150:
# 			return np.argmax(num_matches)
# 		else:
# 			return None
