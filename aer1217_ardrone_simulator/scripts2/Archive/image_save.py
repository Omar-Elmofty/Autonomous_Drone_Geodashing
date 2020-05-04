#!/usr/bin/env python2


"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import


# rospy for the subscriber
import rospy

# ROS Image message
from sensor_msgs.msg import Image

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# OpenCV2 for saving an image
import cv2
import PIL
from PIL import Image as IM


class ImageSave:
    def __init__(self):
        self.sub = rospy.Subscriber('/ardrone/bottom/image_raw', Image, self.image_callback)
        self.image = Image
        self.bridge = CvBridge()
        self.indx = 0
        rate = rospy.Rate(10)
        rospy.wait_for_message('/ardrone/bottom/image_raw', Image)
        while not rospy.is_shutdown():
            
            self.image_save()
            rate.sleep()


    def image_callback(self,msg):
        self.image = msg


    def image_save(self):
    
        # Instantiate CvBridge
        
        
        try:
            # Convert ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
            
            im_pil = IM.fromarray(cv2_img)
            im_pil.save('drone_images/im'+str(self.indx)+'.png')
            self.indx +=1
           
            

        except CvBridgeError, e:
            print('error')


if __name__ == '__main__':
    rospy.init_node('image_listener')
    ImageSave()
