#!/usr/bin/env python

__author__ = 'aivanovic'

import sys, copy
import rospy, tf2_ros, tf_conversions, tf
import cv2
import numpy, math
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

class RgbdImagePublisher:

  def __init__(self):    
    self.rate = rospy.get_param('~rate', 24)
    self.loop_flag = rospy.get_param('~loop_images', True)
    self.frame_counter = 0

    # Configure directories that contain images
    self.directory = rospy.get_param('~directory', "sample_dataset")
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
        depth_file.readline(-1).split(" ")[1].split("\r")[0])
      self.rgb_image_paths.append("/" + 
        rgb_file.readline(-1).split(" ")[1].split("\r")[0])
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
      q_blender[3] = float(gt_string[7].split("\r")[0])
      q_new = tf.transformations.quaternion_multiply(q_blender, q_rotation)
      gt_current.pose.orientation.x = q_new[0]
      gt_current.pose.orientation.y = q_new[1]
      gt_current.pose.orientation.z = q_new[2]
      gt_current.pose.orientation.w = q_new[3]
      self.groundtruth_poses.poses.append(copy.deepcopy(gt_current))
    
    self.ground_truth_pub = rospy.Publisher("ground_truth", PoseStamped, 
      queue_size=1)
    # Tf2 broadcaster for world to camera transform
    self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

    # Publishers for depth and RGB images
    self.bridge = CvBridge()
    self.depth_image_scale = rospy.get_param('~depth_image_scale', 5)
    self.depth_image_pub = rospy.Publisher("depth_image", Image, queue_size=1)
    self.rgb_image_pub = rospy.Publisher("rgb_image", Image, queue_size=1)


    # Camera info based on blender settings
    self.camera_info_pub = rospy.Publisher("camera_info", 
      CameraInfo, queue_size=1)
    self.camera_frame = rospy.get_param('~camera_frame', "camera")
    # Sensor width in mm
    self.camera_sensor_width = rospy.get_param('~camera_sensor_width', 32.0)
    self.camera_focus = rospy.get_param('~camera_focus', 29.47)
    self.res_height = rospy.get_param('~height', 480)
    self.res_width = rospy.get_param('~width', 640)
    self.distortion_model = rospy.get_param('~distortion_model', "plumb_bob")
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


  def run(self):
    rate = rospy.Rate(self.rate)
    self.frame_counter = 0
    while not rospy.is_shutdown():
      rate.sleep()

      timestamp = rospy.Time.now()

      # TODO: Subscribe to loop flag topic
      # TODO: Subscribe to trigger topic or service that sets
      #   self.frame_counter = 0. This will trigger publishing start
      if (self.loop_flag == True) or (self.frame_counter < self.n_frames):
        # Set up current frame index
        i = self.frame_counter % self.n_frames

        # Load depth image
        depth_path = self.directory + self.depth_image_paths[i]
        #img = cv2.imread(depth_path, cv2.IMREAD_GRAYSCALE)
        img = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        #img = img.astype(numpy.uint16)
        img = img/self.depth_image_scale
        img = img.astype(numpy.uint16)
        depth_img = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        # Load rgb image
        rgb_path = self.directory + self.rgb_image_paths[i]
        img2 = cv2.imread(rgb_path, cv2.IMREAD_UNCHANGED)
        #cv2.imshow("window_name", img2)
        #cv2.waitKey(0)
        #img2 = img2.astype(numpy.uint16)
        rgb_img = self.bridge.cv2_to_imgmsg(img2, encoding="bgra8")

        # Publish depth image
        depth_img.header.stamp = timestamp
        depth_img.header.frame_id = self.camera_frame
        self.depth_image_pub.publish(depth_img)

        # Publish rgb img
        rgb_img.header.stamp = timestamp
        rgb_img.header.frame_id = self.camera_frame
        self.rgb_image_pub.publish(rgb_img)

        self.frame_counter = self.frame_counter + 1

      # Publish camera info
      self.camera_info.header.stamp = timestamp
      self.camera_info_pub.publish(self.camera_info)

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


def main(args):
  rospy.init_node('rgbd_image_publisher', anonymous=True)
  blender_to_rgbd = RgbdImagePublisher()
  blender_to_rgbd.run()

if __name__ == '__main__':
    main(sys.argv)