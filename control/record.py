#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose,Twist,TransformStamped,QuaternionStamped
from sensor_msgs.msg import Imu
from tf.msg import tfMessage
import math as M

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:
 
  def __init__(self):
 
    self.bridge   = CvBridge()
    self.event    = rospy.Subscriber("event"   ,Image,self.event_callback)
    self.frame    = rospy.Subscriber("frame"   ,Image,self.frame_callback)
    self.infrared = rospy.Subscriber("infrared",Image,self.infrared_callback)
 
  def event_callback(self,data):
	try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	#(rows,cols,channels) = cv_image.shape
	# if cols > 60 and rows > 60 :
	#  cv2.circle(cv_image, (50,50), 10, 255)

	cv2.imshow("event", cv_image)
	cv2.waitKey(1)

  def frame_callback(self,data):
	try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	#(rows,cols,channels) = cv_image.shape
	# if cols > 60 and rows > 60 :
	#  cv2.circle(cv_image, (50,50), 10, 255)

	cv2.imshow("frame", cv_image)
	cv2.waitKey(1)

  def infrared_callback(self,data):
	try:
		cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	#(rows,cols,channels) = cv_image.shape
	# if cols > 60 and rows > 60 :
	#  cv2.circle(cv_image, (50,50), 10, 255)

	cv2.imshow("infrared", cv_image)
	cv2.waitKey(100)

 
def main():
	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
	rospy.spin()
	except KeyboardInterrupt:
	print("Shutting down")
	cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main()



 
