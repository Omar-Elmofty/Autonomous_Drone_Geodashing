#!/usr/bin/env python2


"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import



import rospy
import numpy as np
import cv2
import copy
import scipy.misc
import PIL
from PIL import Image as IM
import imutils
from target_detect import BlobDetector, CoordinateTransform

from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_matrix
from ransac import RANSAC
from tf.transformations import euler_from_matrix


class LandmarkClassifier(object):
    """Class for classifying landmarks
    """
    def __init__(self):
        #Subscribers
        self.image_sub = rospy.Subscriber('/ardrone/bottom/image_raw', 
                                            Image, self.image_callback)
        self.sub_vicon = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre', 
                           TransformStamped,
                           self.update_quadrotor_state)

        #Global variables
        self.image = Image
        self.bridge = CvBridge()
        self.quad_state = TransformStamped()
        self.T_body2vicon = np.identity(4)

        #wait for msg for initiating processing
        rospy.wait_for_message('/ardrone/bottom/image_raw', Image)

        #time flags for detecting when bag file is done publishing
        self.current_time =  rospy.Time.now().to_sec() 
        self.last_msg_time =  rospy.Time.now().to_sec() 

        #Define landmark locations
        self.landmarks = np.array([[4.32, 8.05],
                                    [0.874, 5.49],
                                    [7.68, 4.24],
                                    [4.27, 1.23]])
        #list of 
        self.landmark_prev_save_rad = [float('inf'), float('inf'), float('inf'),float('inf')]
        self.save_radius = 0.5

        #pose graph
        self.G = {}

        #load images for classification
        img1 = cv2.imread('cn_tower.png') 
        img2 = cv2.imread('casa_loma.png')
        img3 = cv2.imread('nathan_philips_square.png')
        img4 = cv2.imread('princes_gates.png')
        self.landmark_ref_images = [img1, img2, img3, img4]
        self.landmark_names = ['cn_tower', 'casa_loma', 'nathan_philips_square', 'princes_gates']

        #Initialize ransac
        self.ransac = RANSAC()


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

        self.last_msg_time =  rospy.Time.now().to_sec() 


    def update_quadrotor_state(self, msg):
        """Callback for vicon topic
        """
        self.quad_state = msg

    def save_image(self, image, image_name):
        """Function for saving images to file
        """
        im_pil = IM.fromarray(image)
        b, g, r = im_pil.split()
        im_pil = IM.merge("RGB", (r, g, b))
        im_pil.save(image_name + '.png')

    def save_landmark_images(self):
        """Function that saves landmark images from project.bag, images
        that minimizes the distance between the drone and landmarks are
        saved
        """
        #run ros loop to save images
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
            pos = T_b2vic[0:2,3]

            for i in range(self.landmarks.shape[0]):
                #landmark position
                l_pos = self.landmarks[i,:]

                rad = np.linalg.norm(pos - l_pos)
                #save the landmark if it will reduce the distance between drone and landmark
                if rad < self.save_radius and rad <  self.landmark_prev_save_rad[i]:
                    self.save_image(cv2_img, str(i))
                    self.G[i] = T_b2vic             #save pose for that landmark
                    self.landmark_prev_save_rad[i] = rad

    def classify_landmarks(self):
        """Function that classifies the landmarks based on the largest
        set of inliers from ransac
        """

        #Classify Landmarks
        for i in range(self.landmarks.shape[0]):
            #load saved images
            imgq = cv2.imread(str(i)+'.png')

            max_inliers = 0
            for j in range(len(self.landmark_ref_images)):
                #load reference image
                imgt = self.landmark_ref_images[j]

                #perform ransac outlier rejection
                inlier_count, avg_angle = self.ransac.ransac(imgq, imgt)

                #largest set of inliers record
                if inlier_count > max_inliers:
                    best_count = inlier_count
                    best_angle = avg_angle
                    max_inliers = inlier_count
                    best_idx = j

            #calculate landmark orientation
            R = self.G[i][0:3,0:3]
            (phi, theta, psi) = euler_from_matrix(R, 'rxyz')
            landmark_orientation = psi+best_angle

            #print results
            print('Image :'+str(i)+'.png')
            print('location : ', self.landmarks[i])
            print('Landmark :'+self.landmark_names[best_idx])
            print('Landmark Orientation :'+str(landmark_orientation))
            print('inlier_count :'+str(best_count))


if __name__ == '__main__':
    rospy.init_node('Landmark_Classify')
    L = LandmarkClassifier()
    L.save_landmark_images()
    L.classify_landmarks()
