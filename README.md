# SideWalker

[![forthebadge](https://forthebadge.com/images/badges/made-with-python.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/60-percent-of-the-time-works-every-time.svg)](https://forthebadge.com)
[![forthebadge](https://forthebadge.com/images/badges/built-with-love.svg)](https://forthebadge.com)

A [**ROS**](http://wiki.ros.org/noetic) package developed for a course regarding Cybathlon contest's sidewalk challenge.

Tested and reviewed under **ROS-noetic** and **Ubuntu 20.04**, this package, unaltered, depends on the use of an [**Astra Pro**](https://orbbec3d.com/index/Product/info.html?cate=38&id=36) camera made by Orbbec as well as its [ROS1 package](https://github.com/orbbec/ros_astra_camera.git) `ros_astra_camera`.
You will also need the `darknet_ros` [package](https://github.com/leggedrobotics/darknet_ros.git).

The `pre_sidewalk.launch` file starts the launch files from the two previously mentionned packages.
Notice that you need to adjust the followings in order to make it work with the **Astra Pro** camera as well as any other camera/package pair.

```XML
	<!-- includes RGB-D launch file -->
	<include file = "$(find astra_camera)/launch/astra_pro.launch"/>
```


## Authors

Marine Ducrot, Matthieu Lacheny and Axel Lansiaux

