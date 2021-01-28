# blender\_rgbd\_ros

This package is used to convert depth and rgb images, rendered in Blender, to point cloud data. The [BlenderToRGBD](https://github.com/JavonneM/BlenderToRGBD) plugin is used to generate rgb and depth images while rendering an animation. The folder structure for this package is assumed to be the same as generated by the Blender plugin.

## 1. Running the code
To run the node for loading and publishing camera parameters, depth images and rgb images, type:
``` bash
roslaunch blender_rgbd_ros depth_and_rgb_pub.launch
```

<br />

To convert images to the point cloud data, run the [depth\_image\_proc](http://wiki.ros.org/depth_image_proc) pipeline launch file with:
``` bash
roslaunch blender_rgbd_ros depth_image_proc_pipeline.launch
```

## 2. Topic overview
### 2.1 depth\_rgb\_pub.py
#### 2.1.1 Subscribed topics
None so far

#### 2.1.2 Published topics
``depth/image_rect_raw`` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) <br />
  uint16 depth image  <br />

``depth/camera_info`` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)) <br />
  Camera info for depth image  <br />

<br />

``color/image_raw`` ([sensor_msgs/Image](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html)) <br />
  Color image  <br />

``color/camera_info`` ([sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html)) <br />
  Camera info for color image  <br />

<br />

``camera/ground_truth_pose`` ([geometry_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)) <br />
  Camera ground truth position and orientation