#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
import threading

class image_converter:

  def __init__(self):
    print(':::::INIT:::::')
    self.image_pub = rospy.Publisher("RGBD_Image",Image,queue_size=4)
    self.bridge = CvBridge()
    self.rgb_image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callbackRGB)
    self.depth_image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callbackDepth)
    self.combined_img = None
    self.countRGB = 0
    self.countDepth = 0
    self.mutex = 0
    self.mutexOld = 0
    self.semaphore = threading.Lock()
    self.temp = np.zeros((480,1280,3),dtype=np.uint8)
    self.rgbImg = None
    self.depthImg = None

  def callbackRGB(self,data):

    self.semaphore.acquire()
    self.mutex = 1
    if self.mutex != self.mutexOld:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.combined_img = cv_image
      self.rgbImg = cv_image
      self.temp[:,:640] = cv_image
      # print('RGB:---------->',cv_image.shape,self.combined_img.shape)
      self.mutexOld = self.mutex
    self.semaphore.release()
    
  def callbackDepth(self,data):

    self.semaphore.acquire()
    self.mutex = 0
    if self.mutex != self.mutexOld:
      cv_image = self.bridge.imgmsg_to_cv2(data)
      x,y,z = cv_image.copy(),cv_image.copy(),cv_image.copy()
      img = cv2.merge((x,y,z))
      # img = img*255
      img = np.nan_to_num(img)
      self.depthImg = img
      # print(np.amax(img),np.amin(img))
      width = img.shape[1]
      height = img.shape[0]

      #print(img[height//2][width//2])
      self.temp[:,640:] = img

      # np.concatenate((self.combined_img,img))
      # print(self.combined_img.shape,img.shape,self.temp.shape)
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.temp,"bgr8"))
      # cv2.imshow('combined',self.temp)
      # cv2.waitKey(5)
      self.mutexOld = self.mutex
    self.semaphore.release()

def main(args):
  
  rospy.init_node('image_converter', anonymous=False)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)