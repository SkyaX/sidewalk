#!/usr/bin/env python3

import rospy
import struct
import numpy as np
import torch
from cv_bridge import CvBridge
import cv2
import time
import sys
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes as BBs
from darknet_ros_msgs.msg import BoundingBox as BB
import message_filters

class OBJ :
	def __init__(self, bb) :
		self.Bbox = bb

		self.obj_type = bb.Class

		self.xmax = bb.xmax
		self.xmin = bb.xmin
		self.ymax = bb.ymax
		self.ymin = bb.ymin

		self.Xc = int(((self.xmax-self.xmin)/2)+self.xmin)
		self.Yc = int(((self.ymax-self.ymin)/2)+self.ymin)

def get_subs():
	rospy.Subscriber('/darknet_ros/bounding_boxes', BBs,  process_BBs)

	cam_img_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
	depth_img_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
	rospy.Subscriber('/darknet_ros/detection_image', Image,  process_yolo_img, queue_size=2)

	ts = message_filters.ApproximateTimeSynchronizer([cam_img_sub, depth_img_sub], queue_size=10, slop=0.5)
	ts.registerCallback(cluster_RGBD)

def get_pubs(): 
	Traj_publisher = rospy.Publisher('trajectoire', Twist)

def process_BBs(BBS):
	#print("\n---------------------------------------")
	global OBJs

	OBJs = [ OBJ(bb) for bb in BBS.bounding_boxes if bb.probability>0.30 ]

def cluster_RGBD(cam_image, depth_image):
	
	global CAM_IMG 
	global DEP_IMG
	global cv_dep_image
	global OBJs

	DEP_IMG = depth_image
	#print(f"DEP_IMG : {DEP_IMG.header.stamp} / sizeof : {sys.getsizeof(DEP_IMG)}")
	CAM_IMG = cam_image
	#print(f"CAM_IMG : {CAM_IMG.header.stamp} / sizeof : {sys.getsizeof(CAM_IMG)}")

	cv_cam_image = bridge.imgmsg_to_cv2(CAM_IMG, desired_encoding='passthrough')
	cv_dep_image = bridge.imgmsg_to_cv2(DEP_IMG, desired_encoding='passthrough')
	# plt.imshow(cv_dep_image); plt.show()

	cv_dep_rgb = cv2.normalize(cv_dep_image, None, alpha = 0, beta = 255, norm_type = cv2.NORM_MINMAX, dtype=cv2.CV_8U)
	cv_dep_rgb = cv2.cvtColor(cv_dep_rgb, cv2.COLOR_GRAY2RGB, 0);

	Z = cv_cam_image.reshape((-1,3))
	# convert to np.float32
	Z = np.float32(Z)
	# define criteria, number of clusters(K) and apply kmeans()
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
	K = 8
	ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
	# Now convert back into uint8, and make original image
	center = np.uint8(center)
	res = center[label.flatten()]
	res2 = res.reshape((cv_cam_image.shape))
	cv2.imshow('res2',res2); cv2.waitKey(1)

	"""try :
		for object in OBJs :
			coord = (object.Xc, object.Yc)
			cv_cam_image = cv2.circle(cv_cam_image, coord, radius=10, color=(0, 255, 0), thickness=-1)
			cv_dep_rgb = cv2.circle(cv_dep_rgb, coord, radius=10, color=(0, 255, 0), thickness=-1)

	except : print("Waiting for the darknet topic")"""

	cv2.imshow("Cam", cv_cam_image); cv2.waitKey(1)
	cv2.imshow("Depth", cv_dep_rgb); cv2.waitKey(1)

def process_yolo_img(yolo_image):

	global YOLO_IMG
	global cv_yolo_image

	YOLO_IMG = yolo_image
	#print(f"YOLO_IMG : {YOLO_IMG.header.stamp} ------------------------------------------------")

	cv_yolo_image = bridge.imgmsg_to_cv2(YOLO_IMG, desired_encoding='passthrough')
	
	cv2.imshow("Yolo", cv_yolo_image); cv2.waitKey(1)

def process_dir(direction):
	print(direction)

if __name__ == '__main__':
	rospy.init_node('RGB_clustering', anonymous = True)

	global bridge
	bridge = CvBridge()

	get_subs()
	
	rospy.spin()
