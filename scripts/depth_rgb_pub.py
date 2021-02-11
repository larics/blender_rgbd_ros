#!/usr/bin/env python

__author__ = 'aivanovic'

import sys, copy
import rospy, tf2_ros, tf_conversions, tf
import cv2
import numpy as np
import math
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolRequest, \
  SetBoolResponse

class RgbdImagePublisher:

  def __init__(self):    
    self.rate = rospy.get_param('~rate', 24)
    self.loop_flag = rospy.get_param('~loop_images', True)
    self.frame_counter = 0

    # Configure directories that contain images
    self.directory = rospy.get_param('~images_directory', "sample_dataset")
    depth_file = open(self.directory + "/depth.txt", "r")
    rgb_file = open(self.directory + "/rgb.txt", "r")
    groundtruth_file = open(self.directory + "/groundtruth.txt", "r")
    self.n_frames = sum(1 for line in depth_file)
    # Other files must contain the same number of lines
    nrgb = sum(1 for line in rgb_file)
    ngt = sum(1 for line in groundtruth_file)
    if (self.n_frames != nrgb or self.n_frames != ngt):
      print("Files have different length!")
      exit(0)
    # Return to the beginning of files
    depth_file.seek(0)
    rgb_file.seek(0)
    groundtruth_file.seek(0)

    # Save paths of all files
    self.depth_image_paths = []
    self.rgb_image_paths = []
    self.groundtruth_poses = Path()
    q_blender = tf.transformations.quaternion_from_euler(0,0,0)
    q_rotation = tf.transformations.quaternion_from_euler(math.pi,0,0)
    for i in range(self.n_frames):
      self.depth_image_paths.append("/" + 
        depth_file.readline(-1).split(" ")[1].splitlines()[0])
      self.rgb_image_paths.append("/" + 
        rgb_file.readline(-1).split(" ")[1].splitlines()[0])
      gt_string = groundtruth_file.readline(-1).split(" ")
      gt_current = PoseStamped()
      gt_current.pose.position.x = float(gt_string[1])
      gt_current.pose.position.y = float(gt_string[2])
      gt_current.pose.position.z = float(gt_string[3])
      # Rotate camera for 180 degrees around x axis because of 
      # blender representation
      q_blender[0] = float(gt_string[4])
      q_blender[1] = float(gt_string[5])
      q_blender[2] = float(gt_string[6])
      q_blender[3] = float(gt_string[7].splitlines()[0])
      q_new = tf.transformations.quaternion_multiply(q_blender, q_rotation)
      gt_current.pose.orientation.x = q_new[0]
      gt_current.pose.orientation.y = q_new[1]
      gt_current.pose.orientation.z = q_new[2]
      gt_current.pose.orientation.w = q_new[3]
      self.groundtruth_poses.poses.append(copy.deepcopy(gt_current))
    
    self.ground_truth_pub = rospy.Publisher("camera/ground_truth_pose", PoseStamped, 
      queue_size=1)
    # Tf2 broadcaster for world to camera transform
    self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

    # Publishers for depth and RGB images
    self.bridge = CvBridge()
    self.depth_image_scale = rospy.get_param('~depth_image/scale', 5)
    self.depth_image_pub = rospy.Publisher("depth/image_rect_raw", Image, queue_size=1)
    self.rgb_image_pub = rospy.Publisher("color/image_raw", Image, queue_size=1)


    # Camera info based on blender settings
    self.color_camera_info_pub = rospy.Publisher("color/camera_info", 
      CameraInfo, queue_size=1)
    self.depth_camera_info_pub = rospy.Publisher("depth/camera_info", 
      CameraInfo, queue_size=1)
    self.camera_frame = rospy.get_param('~camera_info/frame_id', "camera")
    # Sensor width in mm
    self.camera_sensor_width = rospy.get_param('~camera_info/sensor_width', 32.0)
    self.camera_focus = rospy.get_param('~camera_info/focus', 29.47)
    self.res_height = rospy.get_param('~camera_info/height', 480)
    self.res_width = rospy.get_param('~camera_info/width', 640)
    self.distortion_model = rospy.get_param('~camera_info/distortion_model', "plumb_bob")
    fx = self.camera_focus*self.res_width/self.camera_sensor_width
    fy = fx
    cx = self.res_width/2
    cy = self.res_height/2

    self.camera_info = CameraInfo()
    self.camera_info.header.frame_id = self.camera_frame
    self.camera_info.height = self.res_height
    self.camera_info.width = self.res_width
    self.camera_info.distortion_model = self.distortion_model
    self.camera_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    self.camera_info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    self.camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    self.camera_info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    # Depth image noise parameters
    self.depth_noise_type = rospy.get_param('~depth_image/noise_type', "none")
    self.noise_gauss_mean = rospy.get_param('~depth_image/noise/gauss/mean', 0.0)
    self.noise_gauss_var = rospy.get_param('~depth_image/noise/gauss/variance', 250.0)
    self.noise_salt_pepper_ratio = rospy.get_param('~depth_image/noise/salt_pepper/ratio', 0.1)
    self.noise_salt_pepper_amount = rospy.get_param('~depth_image/noise/salt_pepper/amount', 0.1)
    self.noise_poisson_gain = rospy.get_param('~depth_image/noise/poisson/gain', 15.0)
    self.noise_speckle_gain = rospy.get_param('~depth_image/noise/speckle/gain', 0.01)

    # Subscribers and services go last
    self.trigger_loop_start_service = rospy.Service("start_image_publishing", 
      Trigger, self.triggerLoopStartCallback)
    self.set_loop_flag_service = rospy.Service("set_loop_flag", SetBool,
      self.setLoopFlagServiceCallback)

  def run(self):
    rate = rospy.Rate(self.rate)
    self.frame_counter = 0
    while not rospy.is_shutdown():
      rate.sleep()

      timestamp = rospy.Time.now()

      #   self.frame_counter = 0. This will trigger publishing start
      if (self.loop_flag == True) or (self.frame_counter < self.n_frames):
        # Set up current frame index
        i = self.frame_counter % self.n_frames

        # Load depth image
        depth_path = self.directory + self.depth_image_paths[i]
        #self.depth_image = cv2.imread(depth_path, cv2.IMREAD_GRAYSCALE)
        self.depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        #self.depth_image = self.depth_image.astype(np.uint16)
        self.depth_image = self.depth_image/self.depth_image_scale
        if self.depth_noise_type != "none":
          self.depth_image = self.addNoise(self.depth_noise_type)
        self.depth_image = self.depth_image.astype(np.uint16)
        depth_img_ros = self.bridge.cv2_to_imgmsg(self.depth_image, encoding="passthrough")
        # Load rgb image
        rgb_path = self.directory + self.rgb_image_paths[i]
        img2 = cv2.imread(rgb_path, cv2.IMREAD_UNCHANGED)
        #cv2.imshow("window_name", img2)
        #cv2.waitKey(0)
        #img2 = img2.astype(np.uint16)
        rgb_img = self.bridge.cv2_to_imgmsg(img2, encoding="bgra8")

        # Publish depth image
        depth_img_ros.header.stamp = timestamp
        depth_img_ros.header.frame_id = self.camera_frame
        self.depth_image_pub.publish(depth_img_ros)

        # Publish rgb img
        rgb_img.header.stamp = timestamp
        rgb_img.header.frame_id = self.camera_frame
        self.rgb_image_pub.publish(rgb_img)

        self.frame_counter = self.frame_counter + 1

      # Publish camera info. At this point these two topics are the same.
      self.camera_info.header.stamp = timestamp
      self.color_camera_info_pub.publish(self.camera_info)
      self.depth_camera_info_pub.publish(self.camera_info)

      # Publish ground truth
      current_gt = self.groundtruth_poses.poses[i]
      current_gt.header.frame_id = "world"
      current_gt.header.stamp = timestamp
      self.ground_truth_pub.publish(current_gt)

      # Broadcast tf of ground truth
      t = TransformStamped()
      t.header.stamp = timestamp
      t.header.frame_id = "world"
      t.child_frame_id = self.camera_frame
      t.transform.translation.x = current_gt.pose.position.x
      t.transform.translation.y = current_gt.pose.position.y
      t.transform.translation.z = current_gt.pose.position.z
      t.transform.rotation = copy.deepcopy(current_gt.pose.orientation)
      self.tf2_broadcaster.sendTransform(t)

  def triggerLoopStartCallback(self, req):
    res = TriggerResponse()
    if self.loop_flag == True:
      res.success = False
      res.message = "Node is set in loop mode."
    else:
      self.frame_counter = 0
      res.success = True
      res.message = "Starting with image publish from dataset."

    return res

  def setLoopFlagServiceCallback(self, req):
    res = SetBoolResponse()

    self.loop_flag = req.data
    res.success = True
    if req.data == True:
      res.message = "Starting publishing images in loop."
    else:
      res.message = "Stopping publishing images in loop."

    return res

  def addNoise(self, noise_type):
    # TODO: If this shows up to be slow, don't return the image but rather
    #   change it in this function.
    if noise_type != "none":
      im = np.ceil(self.depth_image/65535.0)
      im = im.astype(np.uint8)
      ret,mask_image = cv2.threshold(im,0,1,cv2.THRESH_BINARY)
      #cv2.imshow('mask', mask_image*100)
      #cv2.waitKey(0)

    if noise_type == "gauss":
      ch = 1
      row = 1
      col = 1
      if (len(self.depth_image.shape) == 2):
        row,col = self.depth_image.shape
      else:
        row,col,ch = self.depth_image.shape
      #mask_image = np.copy(self.depth_image)
      #for i in range(row):
      #  for j in range(col):
      #    if self.depth_image[i,j] != 0:
      #      mask_image[i,j] = 1
      mean = self.noise_gauss_mean
      var = self.noise_gauss_var
      sigma = var**0.5
      if (len(self.depth_image.shape) == 2):
        gauss = np.random.normal(mean,sigma,(row,col))
        gauss = gauss.reshape(row,col)
      else:
        gauss = np.random.normal(mean,sigma,(row,col,ch))
        gauss = gauss.reshape(row,col,ch)
      noisy = self.depth_image + gauss
      noisy = np.multiply(noisy,mask_image)
      return noisy

    elif noise_type == "salt_pepper":
      ch = 1
      row = 1
      col = 1
      if (len(self.depth_image.shape) == 2):
        row,col = self.depth_image.shape
      else:
        row,col,ch = self.depth_image.shape
      s_vs_p = self.noise_salt_pepper_ratio
      amount = self.noise_salt_pepper_amount
      out = np.copy(self.depth_image)
      # Salt mode
      num_salt = np.ceil(amount * self.depth_image.size * s_vs_p)
      coords = [np.random.randint(0, i - 1, int(num_salt))
              for i in self.depth_image.shape]
      out[coords] = 1

      # Pepper mode
      num_pepper = np.ceil(amount* self.depth_image.size * (1. - s_vs_p))
      coords = [np.random.randint(0, i - 1, int(num_pepper)) for i in self.depth_image.shape]
      out[coords] = 0
      out = np.multiply(out,mask_image)
      return out

    elif noise_type == "poisson":
      vals = len(np.unique(self.depth_image))
      vals = self.noise_poisson_gain ** np.ceil(np.log2(vals))
      noisy = np.random.poisson(self.depth_image * vals) / float(vals)
      noisy = np.multiply(noisy,mask_image)
      return noisy

    elif noise_type =="speckle":
      ch = 1
      row = 1
      col = 1
      if (len(self.depth_image.shape) == 2):
        row,col = self.depth_image.shape
      else:
        row,col,ch = self.depth_image.shape
      if (len(self.depth_image.shape) == 2):
        gauss = np.random.randn(row,col)
        gauss = gauss.reshape(row,col)  
      else:
        gauss = np.random.randn(row,col,ch)
        gauss = gauss.reshape(row,col,ch)        
      noisy = self.depth_image + self.noise_speckle_gain*self.depth_image * gauss
      return noisy
    else:
      return self.depth_image

def main(args):
  rospy.init_node('rgbd_image_publisher', anonymous=True)
  blender_to_rgbd = RgbdImagePublisher()
  blender_to_rgbd.run()

if __name__ == '__main__':
    main(sys.argv)