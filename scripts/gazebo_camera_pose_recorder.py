#!/usr/bin/env python

__author__ = 'aivanovic'

import sys, copy, os, getpass
import rospy, tf2_ros, tf_conversions, tf
import cv2
import numpy as np
import math
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped

class CameraPoseRecorder:

  def __init__(self):    
    self.rate = rospy.get_param('~rate', 24)

    self.camera_pose = PoseStamped()
    self.executing_trajectory = 0

    # Open file
    self.directory = rospy.get_param('~directory', "/home/"+getpass.getuser())
    self.filename = rospy.get_param('~filename', "camera_pose")
    extension = ".csv"
    i = 0
    while os.path.exists(self.directory + "/" + self.filename  + extension):
      i = i + 1
      self.filename = self.filename + "_" + str(i)
    self.camera_poses_file = open(self.directory + "/" + self.filename + 
      extension, "w")

    # Subscribers
    rospy.Subscriber('/uav/camera/pose', PoseStamped,
      self.cameraPoseCallback, queue_size=1)
    rospy.Subscriber('/uav/executing_trajectory', Int32,
      self.executingTrajectoryCallback, queue_size=1)

  def run(self):
    rate = rospy.Rate(self.rate)
    t_start = time.time()
    self.camera_poses_file.write("time,pos_x,pos_y,pos_z,q_x,q_y,q_z,q_w")
    while not rospy.is_shutdown():
      rate.sleep()

      # If trajectory is executing start recording
      if self.executing_trajectory == 1:
        line = "\n" + str(time.time()-t_start) + ',' + \
          str(self.camera_pose.pose.position.x) + ',' + \
          str(self.camera_pose.pose.position.y) + ',' + \
          str(self.camera_pose.pose.position.z) + ',' + \
          str(self.camera_pose.pose.orientation.x) + ',' + \
          str(self.camera_pose.pose.orientation.y) + ',' + \
          str(self.camera_pose.pose.orientation.z) + ',' + \
          str(self.camera_pose.pose.orientation.w)
        self.camera_poses_file.write(line)

    self.camera_poses_file.close()

  def cameraPoseCallback(self, msg):
    self.camera_pose = msg

  def executingTrajectoryCallback(self, msg):
    self.executing_trajectory = msg.data

def main(args):
  rospy.init_node('camera_pose_recorder', anonymous=True)
  cam = CameraPoseRecorder()
  cam.run()

if __name__ == '__main__':
    main(sys.argv)