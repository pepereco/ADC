#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
from __future__ import division
import roslib
import sys
import traceback
import rospy
import cv2
import numpy as np
import math
import logging
import socket
import threading
import time
import datetime
import lane_detection_module as ld
import tf
import geometry_msgs
#import control_module as control
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image, Imu, Range
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class parking_detection(object):
    def __init__(self):

      """ROS Subscriptions """
      #self.image_pub = rospy.Publisher("/image_converter/output_video",Image, queue_size=10)
      self.image_sub = rospy.Subscriber("/adc_car/sensors/delock_camera/image_raw",Image,self.cvt_image)
      self.parking_right_distance_pub = rospy.Publisher("/control/back_right_parking", Float64, queue_size=10)

      #self.line_type_pub = rospy.Publisher("/vision/right_line_type", String, queue_size=10)
      #self.cmdVelocityDesvPub = rospy.Publisher('/adc_car/cmd_vel', Twist, queue_size=10)

      """ Variables """
      self.bridge = CvBridge()
      self.twisted_msg=Twist()
      self.latestImage = None
      self.outputImage = None
      self.maskedImage = None
      self.binaryImage = None
      self.channelImage = None
      self.maskedRGBImage = None
      self.processedImage = None
      self.global_fit = None
      self.imgRcvd = False
      self.type_line=""

      # Gazebo Variables

      self.corners = np.float32([[1*640/5, 3.5*480/5], [1*640/5, 2.5*480/5],[4*640/5, 2.5*480/5],[4*640/5, 3.5*480/5]]) #Gazebo Conde track
      self.boundaries = [([100, 100, 100], [150, 150, 150])] #Gazebo Conde track

      # Raspicam Variables
      #self.corners = np.float32([[44,560], [378,450],[902,450],[1215,560]]) #Gazebo Conde track
      #self.corners = np.float32([[15,238], [138,187],[217,187],[238,238]]) #Checkerboard
      #self.corners = np.float32([[15,238], [101,140],[189,140],[297,238]]) #Kitchen
      #self.boundaries = [([13, 102, 13], [53, 244, 34])] #LocalMotors track (HSV - yellow)
      #self.boundaries = [([28, 0, 0], [110, 150, 45])] #Kitchen (HSV - black)

      self.intersectionPoint = 0
      self.speed = 0.15
      self.flag = 0
      self.avg = 0
      self.parking_enable=1
      self.roll=0
      self.pitch=0
      self.yaw=0
      self.state_parking="detect"
      self.timer_start= None
      self.counter = 0
      self.counter_detection=0
      self.first_yaw=0
      self.starting_yaw=0
      self.past_yaw=0

    def cvt_image(self,data):
      try:
        self.latestImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.latestImage = cv2.rotate(self.latestImage, cv2.ROTATE_180)
      except CvBridgeError as e:
        print(e)
      if self.imgRcvd != True:
          self.imgRcvd = True

    def publish(self, image,  bridge,  publisher):
        try:
            #Determine Encoding
            if np.size(image.shape) == 3:
                imgmsg = bridge.cv2_to_imgmsg(image, "bgr8")
            else:
                imgmsg = bridge.cv2_to_imgmsg(image, "mono8")
            publisher.publish(imgmsg)
        except CvBridgeError as e:
            print(e)


    def run(self):

     while True:
         # Only run loop if we have an image
         if self.imgRcvd:
             # step 1: undistort image

             #Define region of interest for cropping
             height = self.latestImage.shape[0]

             width = self.latestImage.shape[1]


             """ Gazebo Conde """
             self.vertices = np.array( [[
                        [4*width/5, 2.5*height/5],
                        [1*width/5, 2.5*height/5],
                        [1*width/5, 3.5*height/5],
                        [4*width/5, 3.5*height/5]
                    ]], dtype=np.int32 )


             self.maskedImage = ld.region_of_interest(self.latestImage, self.vertices)

             # step 2: perspective transform
             self.warpedImage,  _,  _ = ld.perspective_transform(self.maskedImage, self.corners)
             """
             cv2.imshow('wraped',self.warpedImage)
             cv2.waitKey(0)
             cv2.destroyAllWindows()
             """
             # step 3: detect binary lane markings
             #self.binaryImage,  self.channelImage = ld.HLS_sobel(self.warpedImage)
             self.binaryImage = ld.binary_thresh(self.warpedImage,  self.boundaries,  'RGB')     #RGB or HSV

             # step 4: fit polynomials

             if self.global_fit is not None:
                 ploty, fitx, fit = ld.fast_fit_polynomials(self.binaryImage,  self.global_fit)
             else:
                 ploty, fitx, fit = ld.fit_polynomials(self.warpedImage, self.binaryImage)

             self.global_fit = fit

             if self.binaryImage is not None:

                 data = cv2.cvtColor(self.binaryImage, cv2.COLOR_GRAY2BGR)

                 r1, g1, b1 = 255, 255, 255 # Original value
                 r2, g2, b2 = 255, 0, 0 # Value that we want to replace it with

                 red, green, blue = data[:,:,0], data[:,:,1], data[:,:,2]
                 mask = (red == r1) & (green == g1) & (blue == b1)
                 data[:,:,:3][mask] = [b2, g2, r2]

                 output = cv2.bitwise_and(self.warpedImage, self.warpedImage,  mask = self.binaryImage)   #Returns an RGB image

                 _,  src,  dst = ld.perspective_transform(self.latestImage, self.corners)
                 Minv = cv2.getPerspectiveTransform(dst, src)

                 newwarp = cv2.warpPerspective(data, Minv, (self.latestImage.shape[1], self.latestImage.shape[0]))

                 self.maskedRGBImage = cv2.addWeighted(newwarp, 1, self.latestImage, 2.0, 0)

             if (fitx.shape[0] >1 and ((np.max(fitx)- np.min(fitx))> 200)):

                 # step 5: draw lane
                 self.processedImage = ld.render_lane(self.maskedRGBImage, self.corners, ploty, fitx)
                 """
                 cv2.imshow('processed',self.processedImage)
                 cv2.waitKey(0)
                 cv2.destroyAllWindows()
                 """
                 # step 6: print curvature
                 #self.curv = get_curvature(ploty, fitx)

                 # step 6: Calculate Setpoint
                 #pts = np.vstack((fitx,ploty)).astype(np.float64).T
                 #self.avg = ld.movingAverage(self.avg, pts[-1][0],  N=20) # Elegoo Kitchen
                 #self.avg = ld.movingAverage(self.avg, np.average(fitx),  N=1) # Gazebo
                 #self.intersectionPoint = np.array([self.avg])
                 #self.intersectionPoint = np.average(fitx)
                 self.intersectionPoint = fitx[0]
                 #print ("if: ",self.intersectionPoint)
                 self.parking_right_distance_pub.publish(self.intersectionPoint)
                 #print (self.intersectionPoint)
                 # Draw the Setpoint onto the warped blank image

                 # el set point esta nomes en el eix x
                 # step 6: Adjust Motors

		 #self.twisted_msg.linear.x=3.0
 		 #if (abs(desv_sp) < 20):
	            #self.twisted_msg.angular.z=0
		 #else:
		    #self.twisted_msg.angular.z=desv_sp*0.03
		 #self.parking_right_distance_pub.publish(self.intersectionPoint)
		 #self.line_type_pub.publish(self.type_line)
		 #self.cmdVelocity.publish(self.twisted_msg)
                 #self.flag = control.adjustMotorSpeed(self.latestImage,  self.intersectionPoint,  self.speed,  self.cmdVelocityPub, self.cmdVelocityStampedPub, self.flag)
             else:
                 self.global_fit=None

                 self.processedImage = self.latestImage

             self.outputImage = self.processedImage



def main(args):

  rospy.init_node('right_line_detection', anonymous=True)

  ld = parking_detection()

  ld.run()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
