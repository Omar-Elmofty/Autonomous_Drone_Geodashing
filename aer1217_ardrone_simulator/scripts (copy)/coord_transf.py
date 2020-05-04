import numpy as np
import cv2

from tf.transformations import quaternion_matrix

		
class CoordinateTransform():
	def __init__(self):
		self.T_cam2body = np.array([[0,-1, 0, 0],
											 [-1, 0 ,0, 0.0125],
											 [0, 0,-1,-0.025],
											 [0, 0, 0, 1]])
		
		K =np.array([[604.62,0.0,320.5],
						  [0.0,604.62,180.5],
						  [0.0,0.0,1.0]])
		
		self.cam_Kinv = np.linalg.inv(K)
										
	def cam2vicon_transform(self, u, v, T_body2vicon):
		
		T_cam2vicon = T_body2vicon.dot(self.T_cam2body)
		T_vicon2cam = np.linalg.inv(T_cam2vicon)
		
		#point normal representation of the ground plane in the camera frame
		n = T_vicon2cam[0:3,2:3]
		p0 = T_vicon2cam[0:3,3:4]
		
		#calculate c = [x/z, y/z, 1]
		y = np.array([u,v,1]).reshape(-1,1)
		c = self.cam_Kinv.dot(y)
		
		#calculate z
		z = n.T.dot(p0) / float(n.T.dot(c))
		
		p_cam = np.zeros((4,1)) 
		p_cam[0:3,0:1] = z * c
		p_cam[3,0] = 1
		
		#conver p_cam to vicon frame
		p_vicon = T_cam2vicon.dot(p_cam)
		
		return p_vicon
