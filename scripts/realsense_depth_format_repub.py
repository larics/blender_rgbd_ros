#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('blender_rgbd_ros')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/red/camera/depth/image_rect_raw_bgr8",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/red/camera/depth/image_rect_raw/compressed",CompressedImage,self.callback, queue_size=1)

  def callback(self,data):
    print("callback")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
      print("success")
    except CvBridgeError as e:
      print(e)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

  def run(self):
    rospy.spin()

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  ic.run()

if __name__ == '__main__':
    main(sys.argv)