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


def image_callback(msg):
    # Instantiate CvBridge
    bridge = CvBridge()

    
    try:
        # Convert ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save OpenCV2 image as a jpeg 
        cv2.imwrite('~/drone_images/camera_image.jpeg', cv2_img)

def main():
    rospy.init_node('image_listener')
    
    rospy.Subscriber('/ardrone/bottom/image_raw', sensor_msg/Image, image_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
