#!/usr/bin/env python2

"""ROS Node for publishing desired positions."""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np
from numpy import pi
from std_msgs.msg import Int16
import math
import cv2

# Import class that computes the desired positions
# from aer1217_ardrone_simulator import PositionGenerator

from geometry_msgs.msg import TransformStamped


class ROSDesiredPositionGenerator(object):
  """ROS interface for publishing desired positions."""
  # write code here for desired position trajectory generator
  def __init__ (self):

    # Publishers
    self.pub_des_pos = rospy.Publisher('/aer1217/desired_position', 
                                      TransformStamped, queue_size=32)
    #Subscriber
    self.move_type = rospy.Subscriber('/aer1217/move_type', 
                                    Int16,
                                    self.update_trajectory)
    #Variables initialization
    self.desired_position_msg = TransformStamped()

    #initializing variables for survey move 
    self.point1 = np.array([-2, -2, 1])
    self.point2 = np.array([-2, 2, 1])
    
    self.position = self.point1
    self.direction = 1

    self.z_input = 1

    #define grid_square distance moved in x direction
    self.grid_square = [math.tan(32/180.0*pi)*self.z_input]   


    #variables for circular move
    self.center_x = 0
    self.center_y = 1
    self.angle = np.radians(0)   
    self.radius = 1           
    self.z_min = 2     
    self.z_max = 2

  
    #frequency for publishing commands
    self.freq = 50

    #initializing input move type 
    self.move_type = 1

  def linear_move(self):
    """Function that creates and publishes a linear trajectory 
    starting from self.point1 to self.point2
    """
    #define delta distance moved every timestep
    delta =  (self.point2-self.point1)/500.0

    #increment position by delta
    self.position = self.position + self.direction * delta
    
    #pls look into this. i couldnt figure out which command to use to stop the drone from moving when reaching the end. 
    #if (self.position >= np.array([2,2,1]) or self.position >= np.array([2,-2,1])
	#   exit(), break, return??? 

    #reverse direction when reaching the ends of trajectory
    if (self.position[1] >= self.point2[1]):

        self.position = self.grid_square + self.point2[0]
    	self.point2= self.position
    	self.point1 = self.point1[0] + self.grid_square
    	self.direction = -1* self.direction

    elif (self.position[1] <=self.point1[1]):
    	self.position = self.grid_square + self.point1[0]
    	self.point1 = self.position
    	self.point2 = self.point2[0] + self.grid_square
      	self.direction = -1*self.direction

    #set positions
    self.desired_position_msg.transform.translation.x = self.position [0]
    self.desired_position_msg.transform.translation.y = self.position [1]
    self.desired_position_msg.transform.translation.z = self.position [2]
    self.desired_position_msg.transform.rotation.z = 0
    self.desired_position_msg.header.stamp = rospy.Time.now()

    #publish positions
    self.pub_des_pos.publish(self.desired_position_msg)


  def circular_move(self):
    """Function that creates and publishes a circular trajectory 
    with a radius of 1
    """
    
    #Increment angle 
    self.angle += 2*np.pi/500.0
    
    #set x and y positions
    x = self.center_x + self.radius * np.cos(self.angle)  
    y = self.center_y + self.radius * np.sin(self.angle) 
   
    #set z position
    if self.angle <= pi:  
      z = self.z_min+(self.z_max - self.z_min)*(1 / pi * self.angle)   
    else:         
      z = self.z_max - (self.z_max - self.z_min)*(1 / pi * (self.angle - pi))
      if self.angle >= 2 * pi:
          self.angle = 0

    #set yaw angle
    psi = self.angle + np.pi

    #Ensure psi lies between -pi and pi
    if psi > np.pi:
        psi = psi - 2*np.pi 
    elif psi < -np.pi:
        psi = psi + 2*np.pi 

    #write desired position message
    self.desired_position_msg.transform.translation.x = x
    self.desired_position_msg.transform.translation.y = y
    self.desired_position_msg.transform.translation.z = z
    self.desired_position_msg.transform.rotation.z = psi
    self.desired_position_msg.header.stamp = rospy.Time.now()

    #publish desired position message
    self.pub_des_pos.publish(self.desired_position_msg)


  def send_des_position(self):
    """Function that runs linear or circular move based on the input
    """
    
    rate = rospy.Rate(self.freq)

    while not rospy.is_shutdown():

      if self.move_type == 0:
        self.linear_move()
      elif self.move_type == 1:
        self.circular_move()

      rate.sleep()


  def update_trajectory(self,int_msg):
    self.move_type = int_msg.data

if __name__ == '__main__':
    rospy.init_node('desired_position')
    pos_generator = ROSDesiredPositionGenerator()
    pos_generator.send_des_position()
