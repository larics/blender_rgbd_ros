#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher:

  def __init__(self):
    self.camera_info_pub = rospy.Publisher("camera_info", 
      CameraInfo, queue_size=1)
    self.rate = rospy.get_param('~rate', 30)
    self.camera_frame = rospy.get_param('~camera_frame', "camera")
    # Sensor width in mm
    self.camera_sensor_width = rospy.get_param('~camera_sensor_width', 32.0)
    self.camera_focus = rospy.get_param('~camera_focus', 29.97)
    self.res_height = rospy.get_param('~height', 480)
    self.res_width = rospy.get_param('~width', 640)
    fx = self.camera_focus*self.res_width/self.camera_sensor_width
    fy = fx
    cx = self.res_width/2 + 0.5
    cy = self.res_height/2 + 0.5

    self.camera_info = CameraInfo()
    self.camera_info.header.frame_id = self.camera_frame
    self.camera_info.height = self.res_height
    self.camera_info.width = self.res_width
    self.camera_info.distortion_model = "plumb_bob"
    self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    self.camera_info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    self.camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    self.camera_info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]


  def run(self):
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()
      self.camera_info.header.stamp = rospy.Time.now()

      self.camera_info_pub.publish(self.camera_info)


if __name__ == '__main__':

  rospy.init_node('blender_camera_info_pub')
  cam = CameraInfoPublisher()
  cam.run()
