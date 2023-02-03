# SideWalker

[![forthebadge](https://forthebadge.com/images/badges/made-with-python.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/60-percent-of-the-time-works-every-time.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/built-with-love.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/it-works-why.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/works-on-my-machine.svg)](https://forthebadge.com)

A [**ROS**](http://wiki.ros.org/noetic) package, relying on the RGB-D camera technology, developed for a course regarding the Cybathlon contest's sidewalk challenge.

Its purpose is to identify obstacles to avoid, within a certain margin of error, and isolate the optimal direction for a visually impared person to follow.

Tested and reviewed under **ROS-noetic** and **Ubuntu 20.04**, this package, unaltered, depends on the use of an [**Astra Pro**](https://orbbec3d.com/index/Product/info.html?cate=38&id=36) camera made by Orbbec as well as its [ROS1 package](https://github.com/orbbec/ros_astra_camera.git) `ros_astra_camera`.

![astra_pro](https://github.com/SkyaX/sidewalk/blob/main/images/Astra%20Pro%20Plus_00.png)&nbsp;

You will also need the `darknet_ros` [package](https://github.com/leggedrobotics/darknet_ros.git).

## Installation

Make sure that the two previously mentionned dependencies are downloaded in your catkin workspace.

 - Once everything is checked and working, clone the latest version of the repository in the `src` folder of your workspace and build the newly added packages.

```bash
cd catkin_ws/src
git clone https://github.com/SkyaX/sidewalk.git
cd ..
catkin_make
```

The `pre_sidewalk.launch` file starts the launch files from the two previously mentionned packages.
Notice that you need to adjust the followings in order to make it work with the **Astra Pro** camera as well as any other camera/package pair :

 - The included `astra_pro.launch` used by our particular model : 

```XML
	<!-- includes RGB-D launch file -->
	<include file = "$(find astra_camera)/launch/astra_pro.launch"/>
```

 - The included `darknet_ros.launch` altered as follows : 

```XML
 	<!-- includes darknet launch file --> 
	<include file = "$(find darknet_ros)/launch/darknet_ros.launch"/> 
```

![image](https://github.com/SkyaX/sidewalk/blob/main/images/darknet_ros_launch.png "Darknet_ros launch file")

The 6th line may be adjusted depending on the topic in which the **RGB** image from your camera is published.

 - Note that we used _yoloV7-tiny_, which does not come ready with `darknet_ros`. 
After downloading the `darknet_ros` package, you can simply add the [`yolov7.yaml`](https://github.com/SkyaX/sidewalk/blob/main/configs/yolov7.yaml) file in the `darknet_ros/config` folder.
Then change the 14th line of the previously mentioned file from `yolov3.yaml` -> `yolov7.yaml`.
Finally, you can add the [yolov7-tiny.weights](https://github.com/SkyaX/sidewalk/blob/main/configs/yolov7-tiny.weights) & [yolov7-tiny.cfg](https://github.com/SkyaX/sidewalk/blob/main/configs/yolov7-tiny.cfg) in the `darknet_ros/yolo_network_config/weights` & `darknet_ros/yolo_network_config/cfg` folders, respectively.

## Usage

Here is an overview of the system's organisation :

![rqtgraph](https://github.com/SkyaX/sidewalk/blob/main/images/rosgraph.png)

 - Firstly, make sure that the **Subscribers** in the file [`trajectoire_final.py`](https://github.com/SkyaX/sidewalk/blob/main/src/trajectoire_final.py) are correctly created. They should be given the correct topic depending on the package you use for your camera, as follows :

 ```python
	cam_img_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
	depth_img_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
 ```

 The library `message_filters` is used to assure the correct synchronization between the depth and RGB images.

### Launch

 - Start by launching the `pre_sidewalk.launch` file :
```bash
roslaunch sidewalk pre_sidewalk.launch
```

 - You can then launch the main file :
```bash
roslaunch sidewalk opti_sidewalk.launch
```

The different OpenCV outputs :
<img src="https://github.com/SkyaX/sidewalk/blob/main/images/yolo_gif.gif" align="right" width="35%"/>


## Authors

Marine Ducrot, Matthieu Lacheny and Axel Lansiaux

