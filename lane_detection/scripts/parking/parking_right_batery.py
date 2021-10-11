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
from iri_model_car_msgs.msg import encoders
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class parking_detection(object):
    def __init__(self):
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

      self.corners = np.float32([[2.7*640/5, 3.5*480/5], [2.5*640/5, 3*480/5],[4.35*640/5, 3*480/5],[5*640/5, 3.5*480/5]]) #Gazebo Conde track
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
      self.parking_enable=0
      self.parking_out_enable=1
      self.roll=0
      self.pitch=0
      self.yaw=0
      self.state_parking="detect"
      self.state_parking_out = "out"
      self.timer_start= None
      self.counter = 0
      self.counter_detection=0
      self.counter_detection_2=0
      self.first_yaw=0
      self.starting_yaw=0
      self.past_yaw=None
      self.reference_yaw=None
      self.right_encoder_start=0
      self.right_encoder=0
      self.left_encoder=0
      self.control_alignment=""
      self.counter_center=0
      self.time_start=None
      self.sonar_right=None
      self.encoders_past_detected_line=0
      self.encoders_diff_lines=0
      self.sonar_right_avg=0

      """ROS Subscriptions """
      #self.image_pub = rospy.Publisher("/image_converter/output_video",Image, queue_size=10)
      self.image_sub = rospy.Subscriber("/adc_car/sensors/basler_camera/image_raw",Image,self.cvt_image)
      self.parking_right_distance_pub = rospy.Publisher("/control/front_right_parking", Float64, queue_size=10)
      self.imu_sub = rospy.Subscriber("/adc_car/sensors/imu_data", Imu, self.imu_callback)
      self.pub_park= rospy.Publisher('/laser/park', String, queue_size=10)
      self.pub_state= rospy.Publisher('/parking/state', String, queue_size=10)
      #self.sub_back_sonar=rospy.Subscriber('/adc_car/sensors/range/rear_center_sonar', Range, self.callback_back_sonar)
      self.sub_right_sonar=rospy.Subscriber('/adc_car/sensors/range/side_right_sonar', Range, self.callback_right_sonar)
      self.sub_encoders=rospy.Subscriber('/adc_car/encoders', encoders, self.cb_encoders)
      self.sub_control_align=rospy.Subscriber('/control/alignment', String, self.callback_control_alignment)
      self.sub_back_camera=rospy.Subscriber('/control/back_right_parking', Float64, self.cb_back_camera)
      self.sub_back_camera=rospy.Subscriber('/xodr/parking_enable', String, self.cb_enable)
      self.sub_detect=rospy.Subscriber('/parking/detect', String, self.cb_enable)
      self.pub_detect= rospy.Publisher('/parking/detect', String, queue_size=10)
      #self.line_type_pub = rospy.Publisher("/vision/right_line_type", String, queue_size=10)
      #self.cmdVelocityDesvPub = rospy.Publisher('/adc_car/cmd_vel', Twist, queue_size=10)

    def cb_enable(self, msg):
        if msg.data == "right":
            self.parking_enable=1
        if msg.data =="right_detect" or msg.data =="left_detect":
            print ("park right bat deactiveted")
            self.parking_enable=0

    def callback_right_sonar(self, msg):
        self.sonar_right=msg.range
        self.sonar_right_avg = ld.movingAverage(self.sonar_right_avg, self.sonar_right,  N=5)
        #print ("sonar: ", self.sonar_right)
        #print ("sonar avg: ", self.sonar_right_avg)
    def callback_control_alignment(self,msg):
        self.control_alignment=msg.data

    def imu_callback(self, data):
        orientation_list_quaternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (self.roll, self.pitch, self.yaw) = tf.transformations.euler_from_quaternion(orientation_list_quaternion)
        #self.yaw = self.yaw_1+ 3
        #print (self.yaw)
        if self.parking_enable==1 and self.past_yaw!=None :
            if (abs(self.yaw - self.past_yaw)> 2):
                if self.reference_yaw==None:
                    self.reference_yaw=self.past_yaw
                    #print ("reference: ", self.reference_yaw)
                self.yaw = math.copysign(abs(self.reference_yaw)+abs(abs(self.yaw)-abs(self.reference_yaw)) ,self.reference_yaw)
                #print ("difference: ", abs(self.reference_yaw)+abs(abs(self.yaw)-abs(self.reference_yaw)))
            self.past_yaw=self.yaw
            self.state_machine_parking()
        else:
            self.past_yaw=self.yaw
        if self.parking_out_enable ==1 :
            self.state_machine_parking_out()

    def cb_back_camera(self, data):
        self.back_intersection_point = data.data
        #print ("back: ",self.back_intersection_point)

    def cb_encoders(self, msg):
        self.right_encoder = msg.right_ticks
        self.left_encoder = msg.left_ticks
        #print ("encoders right: ",self.right_encoder)

    def state_machine_parking(self):
        self.pub_state.publish("parking")
        if self.state_parking == "detected":
            #self.timer_start = time.time()
            print ("sonar bat", self.sonar_right_avg)
            print ("encoders diff : ", self.encoders_diff_lines)
            if self.sonar_right_avg > 2.3 and 25<self.encoders_diff_lines < 55:
                self.pub_detect.publish("right_batery_detect")
                print ("park right bat detected")
                print ("im in avance")
                self.right_encoder_start = self.right_encoder
                self.state_parking = "avance"
                #self.state_parking= "detect"
            else:
                self.counter=0
                self.state_parking= "detect"
        elif self.state_parking== "avance":
            """if time.time()-self.timer_start < 18:
                self.pub_park.publish("front")"""
            #if (self.right_encoder-self.right_encoder_start < 85):
            if (self.right_encoder-self.right_encoder_start < 55):
                print ("encoders: ", self.right_encoder-self.right_encoder_start)
                print ("intersection: ", self.intersectionPoint)
            else:
                print ("counter: ", self.counter_detection)
                self.pub_park.publish("stop")
                time_before = time.time()
                while time.time() -time_before < 1:
                    pass
                self.first_yaw=self.yaw
                self.past_yaw=self.yaw
                self.starting_yaw = self.yaw
                self.right_encoder_start=self.right_encoder
                self.counter=0
                self.state_parking="park_1"
        elif self.state_parking == "park_1":
            #print ("front cam : ", self.intersectionPoint)
            #print ("park_1: ",abs(self.yaw - self.starting_yaw ))
            #print ("self yaw: ", self.yaw)
            #print ("starting yaw: ", self.starting_yaw)
            #print ("encoders diff: ", self.right_encoder-self.right_encoder_start)
            #self.pub_park.publish("right_back")
            #print ("back cam: ", self.back_intersection_point)
            #if ((abs(self.yaw - self.starting_yaw ) > 0.5) and (self.intersectionPoint>570) and (self.right_encoder-self.right_encoder_start)> 48) :
            #if (self.intersectionPoint>580) and (abs(self.yaw - self.starting_yaw ) > 0.7) :
            if (self.right_encoder-self.right_encoder_start)> 75:
                self.counter+=1
                if self.counter>2:
                    print ("stop")
                    self.pub_park.publish("stop")
                    time_before = time.time()
                    while time.time() -time_before < 1:
                        pass
                    self.right_encoder_start=self.right_encoder
                    self.state_parking="park_2"

            else:
                #print ("back_right")
                self.pub_park.publish("left_back")
        elif self.state_parking =="park_2":
            print ("encoders diff: ", self.right_encoder-self.right_encoder_start)
            #if not(1200 < self.back_intersection_point < 1600):
            if (self.right_encoder-self.right_encoder_start) < 20:
                #self.counter=0
                self.pub_park.publish("right_front")
            else:
                if (self.right_encoder-self.right_encoder_start) < 67:
                    self.pub_park.publish("stop")
                    self.pub_park.publish("end")
                else:
                    self.pub_park.publish("stop")
                    self.state_parking="parked"


    def state_machine_parking_out(self):
        self.pub_state.publish("unparking")
        if self.state_parking_out=="out" and self.state_parking=="parked":
            self.right_encoder_start=self.right_encoder
            self.state_parking_out="out_1"
        elif self.state_parking_out=="out_1":
            if (self.right_encoder -self.right_encoder_start > 65) :
                self.counter+=1
                if self.counter>5:
                    """print ("stop")
                    self.pub_park.publish("stop")
                    time_before = time.time()
                    while time.time() -time_before < 1:
                        pass"""
                    self.right_encoder_start= self.right_encoder
                    self.state_parking_out="out_2"
            else:
                self.pub_park.publish("right_back")
        elif self.state_parking_out == "out_2" :
            if (self.right_encoder -self.right_encoder_start) < 20 :
                self.pub_park.publish("alignment_info")
                print(self.control_alignment)
                if self.control_alignment != "center":
                    self.counter_center=0
                    self.pub_park.publish("right_back")
                else:
                    self.counter_center+=1
                    if self.counter_center==4:
                        self.pub_park.publish("stop")
                        time_before = time.time()
                        while time.time() -time_before < 2:
                            self.pub_park.publish("left_front")
                        self.pub_park.publish("stop")
                        self.pub_park.publish("end")
                        self.state_parking_out="out_done"
            else:
                self.pub_park.publish("alignment_info")
                print(self.control_alignment)
                if self.control_alignment != "center":
                    self.counter_center=0
                    self.pub_park.publish("left_front")
                else:
                    self.counter_center+=1
                    if self.counter_center==4:
                        self.pub_park.publish("stop")
                        self.pub_park.publish("end")
                        self.state_parking_out="out_done"

        elif self.state_parking_out== "out_done":
            self.parking_enable=0
            self.state_parking="detect"
            self.state_parking_out="out"


    def cvt_image(self,data):
      try:
        self.latestImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
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
                        [4.35*width/5, 3*height/5],
                        [2.5*width/5, 3*height/5],
                        [2.7*width/5, 3.5*height/5],
                        [5*width/5, 3.5*height/5]
                    ]], dtype=np.int32 )
             """

             # Raspicam
             original
             self.vertices = np.array( [[
                        [2.75*width/5, 3*height/5],
                        [2.25*width/5, 3*height/5],
                        [.5*width/5, height],
                        [4.5*width/5, height]
                    ]], dtype=np.int32 )

             self.vertices = np.array( [[
                        [3.75*width/5, 2*height/5],
                        [1.25*width/5, 2*height/5],
                        [.05*width/5, height],
                        [4.95*width/5, height]
                    ]], dtype=np.int32 )
             """

             self.maskedImage = ld.region_of_interest(self.latestImage, self.vertices)

             # step 2: perspective transform
             self.warpedImage,  _,  _ = ld.perspective_transform(self.maskedImage, self.corners)
             #cv2.imshow('wraped right',self.warpedImage)
             #cv2.waitKey(0)
             #cv2.destroyAllWindows()
             # step 3: detect binary lane markings
             #self.binaryImage,  self.channelImage = ld.HLS_sobel(self.warpedImage)
             self.binaryImage = ld.binary_thresh(self.warpedImage,  self.boundaries,  'RGB')     #RGB or HSV

             # step 4: fit polynomials

             """if self.global_fit is not None:
                 ploty, fitx, fit = ld.fast_fit_polynomials(self.binaryImage,  self.global_fit)
             else:"""
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
                 if self.intersectionPoint < 0 or 2500>self.intersectionPoint > 800:
                     if self.intersectionPoint <0:
                         self.counter_detection += 1
                     else:
                         self.counter_detection+=2
                     #self.counter_detection += 1
                     self.counter_detection_2=0
                 else:
                     self.counter_detection_2+=1
                     if self.counter_detection_2>10:
                         self.counter_detection = 0
                 #print ("if: ",self.intersectionPoint)
                 #print (self.intersectionPoint)
                 # Draw the Setpoint onto the warped blank image
                 """radius = 10
                 cv2.circle(self.processedImage,(int(pts[0][0]), int(pts[0][1])), radius, (255,0,0), -1)
                 cv2.imshow('point',self.processedImage)
                 cv2.waitKey(0)
                 cv2.destroyAllWindows()"""
             else:
                 self.global_fit=None
                 self.processedImage = self.latestImage

                 if self.parking_enable==1 and self.state_parking == "detect":
                     #print ("detecting")
                     #print ("counter: ", self.counter_detection)

                     if self.counter_detection > 10:
                         self.counter_detection=0
                         print ("detected")
                         self.encoders_diff_lines = self.right_encoder-self.encoders_past_detected_line
                         self.encoders_past_detected_line=self.right_encoder
                         self.state_parking= "detected"
                         self.state_machine_parking()
                 self.outputImage = self.processedImage


             #self.publish(self.outputImage, self.bridge,  self.image_pub)


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
