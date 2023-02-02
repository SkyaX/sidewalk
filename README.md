# SideWalker

[![forthebadge](https://forthebadge.com/images/badges/made-with-python.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/60-percent-of-the-time-works-every-time.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/built-with-love.svg)](https://forthebadge.com)

A [**ROS**](http://wiki.ros.org/noetic) package developed for a course regarding Cybathlon contest's sidewalk challenge.

Tested and reviewed under **ROS-noetic** and **Ubuntu 20.04**, this package, unaltered, depends on the use of an [**Astra Pro**](https://orbbec3d.com/index/Product/info.html?cate=38&id=36) camera made by Orbbec as well as its [ROS1 package](https://github.com/orbbec/ros_astra_camera.git) `ros_astra_camera`.
You will also need the `darknet_ros` [package](https://github.com/leggedrobotics/darknet_ros.git).

## Installation

Make sure that the two previously mentionned dependencies are built.

The `pre_sidewalk.launch` file starts the launch files from the two previously mentionned packages.
Notice that you need to adjust the followings in order to make it work with the **Astra Pro** camera as well as any other camera/package pair :

 - The included `astra_pro.launch` used by our particular model : 

```XML
	<!-- includes RGB-D launch file -->
	<include file = "$(find astra_camera)/launch/astra_pro.launch"/>
```

 - The included `darknet_ros.launch` altered as follows : 

![image](https://github.com/SkyaX/sidewalk/blob/main/images/darknet_ros_launch.png "Employee Data title")

Note : We used _yoloV7-tiny_, which does not come ready in `darknet_ros`. 

After downloading the `darknet_ros` package, you can simply add the [`yolov7.yaml`](https://github.com/SkyaX/sidewalk/blob/main/configs/yolov7.yaml) file in the `darknet_ros/config` folder.

Then change the 14th line of the previously mentioned file from `yolov3.yaml` -> `yolov7.yaml`.

Finally, you can add the [yolov7-tiny.weights](https://github.com/SkyaX/sidewalk/blob/main/configs/yolov7-tiny.weights) & [yolov7-tiny.cfg](https://github.com/SkyaX/sidewalk/blob/main/configs/yolov7-tiny.cfg) in the `darknet_ros/yolo_network_config/weights` & `darknet_ros/yolo_network_config/cfg` folders, respectively.


## Authors

Marine Ducrot, Matthieu Lacheny and Axel Lansiaux

