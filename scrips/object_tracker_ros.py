#!/usr/bin/env python


import sys 
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rect_msgs.msg import cvRect
from nav_msgs.msg import Odometry 

import dlib
import cv2


from rect_msgs.msg import cvRect

class object_tracker_ros:
      
      def __init__(self):
          rospy.init_node("object_tracker_ros",anonymous=True)
          self.image_sub = rospy.Subscriber("/realsense_plugin/camera/color/image_raw",Image,self.imgCallback)
          self.rect_pro_pub = rospy.Publisher("/cvRect_track",cvRect,queue_size = 1)
          self.rect_raw_sub = rospy.Subscriber("/cvRect",cvRect,self.rectCallback)
          rate = rospy.Rate(30)
          self.bridge = CvBridge()
          self.pt=[]
          self.cv_image = np.array([])
          self.init = False
          self.tracker = dlib.correlation_tracker()

          self.minx = 0 
          self.miny = 0
          self.maxx = 0
          self.maxy = 0
          self.lock = 1
          print("begin!")
          while not rospy.is_shutdown():
                rate.sleep()
                if(self.init==False):
                   continue
                if(self.cv_image.size > 0):
                   self.tracker.update(self.cv_image)
                   im_ = self.cv_image.copy()
                   rect_ = self.tracker.get_position()
                   pt1 = (int(rect_.left()), int(rect_.top()))
                   pt2 = (int(rect_.right()), int(rect_.bottom()))

                   if(self.lock == 0):
                      self.lock = 1
                      pt3 = (self.minx,self.miny)
                      pt4 = (self.maxx,self.maxy)

                      x1x = max(pt1[0],pt3[0])
                      x1y = max(pt1[1],pt3[1])

                      x2x = min(pt2[0],pt4[0])
                      x2y = min(pt2[1],pt4[1])

                      if(x1x - x2x > 20 or x1y - x2y > 20):
                         self.init = False
                         print("reinit!1")
                         continue
                     
                   
#                      if(x2x > x1x and x2y >x1y):
#                         AJoin = (x2x - x1x)*(x2y -x1y)
#                      else:
#                         self.init = False
#                         print("reinit!1")
#                         continue
#                      self.lock = 1

                   #A1 = (pt2[0] - pt1[0])*(pt2[1] - pt1[1])
                   #A2 = (pt4[0] - pt3[0])*(pt4[1] - pt3[1])

                   #AUnion = (A1 + A2 - AJoin)
                   #same_pre = AJoin/AUnion
                   #if(same_pre < 0.5):
                   #   self.init = False
                   #   print("reinit!2")
                   #   continue
                   
                   cvRect_pub = cvRect()
                   cvRect_pub.x = pt1[0]
                   cvRect_pub.y = pt1[1]
                   cvRect_pub.width = pt2[0] - pt1[0]
                   cvRect_pub.height = pt2[1] - pt1[1]
                   self.rect_pro_pub.publish(cvRect_pub)

                  
                   cv2.rectangle(im_, pt1, pt2, (255, 255, 0), 3)
                   cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
                   cv2.imshow("Image", im_)
                   cv2.waitKey(1)

      def rectCallback(self,msg):
          self.lock = 0
          self.pt=[]
          self.minx =int(msg.x)
          self.miny =int(msg.y)
          self.maxx =int(msg.x + msg.width)
          self.maxy =int(msg.y + msg.height)
          self.pt.append((self.minx,self.miny,self.maxx,self.maxy))
          if(self.init == False and self.cv_image.size > 0):
             self.tracker.start_track(self.cv_image, dlib.rectangle(*self.pt[0]))
             self.init = True
      def imgCallback(self,msg):
          try:
              self.cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
              #print("cv_image: ",self.cv_image.size)
          except CvBridgeError as e:
              print(e)

          

          

    




if __name__ == "__main__":
   try:
      ic = object_tracker_ros()
   except rospy.ROSInterruptException:
      pass
