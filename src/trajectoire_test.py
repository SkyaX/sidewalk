#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
import time

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes as BBs
from darknet_ros_msgs.msg import BoundingBox as BB
import message_filters

class OBJ :
	def __init__(self, bb) :
		self.Bbox = bb

		self.obj_type = bb.Class # Object's Class

		self.xmax = bb.xmax # Object's pos
		self.xmin = bb.xmin
		self.ymax = bb.ymax
		self.ymin = bb.ymin
		self.Xc = int(((self.xmax-self.xmin)/2)+self.xmin) # Center of the object's bounding box
		self.Yc = int(((self.ymax-self.ymin)/2)+self.ymin)

		self.median_depth = 0 # Median depth value

	def __lt__(self, other): # looks at the median_depth if two objects are being compared
		return self.median_depth < other.median_depth


def get_subs():
	rospy.Subscriber('/darknet_ros/bounding_boxes', BBs,  process_BBs)
	
	cam_img_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
	depth_img_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
	rospy.Subscriber('/darknet_ros/detection_image', Image,  process_yolo_img, queue_size=2)

	ts = message_filters.ApproximateTimeSynchronizer([cam_img_sub, depth_img_sub], queue_size=10, slop=0.5)
	ts.registerCallback(process_RGBD)

def process_BBs(BBS):
	""" 
		Processes the bounding boxes from darknet_ros' yolov2/3/7
	"""
	global OBJs
	OBJs = [ OBJ(bb) for bb in BBS.bounding_boxes if (bb.probability>0.30 and bb.Class not in ["refrigerator","mouse"]) ]

def process_RGBD(cam_image, depth_image):
	""" 
		Processes the RGB-D images from astra_pro.launch
	"""
	global cv_dep_image
	global counting_points
	global pointsX
	global pointsY
	global Xc_moy
	global Yc_moy

	DEP_IMG = depth_image
	CAM_IMG = cam_image
	cv_cam_image = bridge.imgmsg_to_cv2(CAM_IMG, desired_encoding='passthrough')
	cv_dep_image = bridge.imgmsg_to_cv2(DEP_IMG, desired_encoding='passthrough')

	kernel = np.ones((15,15), np.uint8)
	dep_dilated = cv2.dilate(cv_dep_image, kernel)

	try : # goes throught the detected objects in proximity order and fills their bbox area with their median depth value
		yolo_means = dep_dilated.copy()
		for object in OBJs :
			object_img = yolo_means[object.ymin:object.ymax,object.xmin:object.xmax]
			object.median_depth = np.median(np.reshape(object_img, (1,-1)))

		for object in np.sort(OBJs)[::-1] : 
			#print(f"{object.obj_type} : {object.median_depth}")
			yolo_means[object.ymin:object.ymax,object.xmin:object.xmax] = object.median_depth

	except : print("Err : try yolo_means")

	cv_dep_norm = cv2.normalize(dep_dilated, None, alpha = 0, beta = 255, norm_type = cv2.NORM_MINMAX, dtype=cv2.CV_8U)
	cv_dep_rgb = cv2.cvtColor(cv_dep_norm, cv2.COLOR_GRAY2RGB, 0);

	yolo_means_norm = cv2.normalize(yolo_means, None, alpha = 0, beta = 255, norm_type = cv2.NORM_MINMAX, dtype=cv2.CV_8U)
	yolo_means_rgb = cv2.cvtColor(yolo_means_norm, cv2.COLOR_GRAY2RGB, 0);

	try : # goes throught the detected objects and displays their centroid
		for object in OBJs :
			coord = (object.Xc, object.Yc)
			cv_cam_image = cv2.circle(cv_cam_image, coord, radius=10, color=(0, 255, 0), thickness=-1)
			cv_dep_rgb = cv2.circle(cv_dep_rgb, coord, radius=10, color=(0, 255, 0), thickness=-1)

	except : print("Err : try green circles")

	highest_median_half = 0
	try : 
		half = np.hsplit(yolo_means_norm, 2)
		medians = [np.mean(col) for col in half]
		highest_median_half = np.argmax(medians)
		if highest_median_half == 0 : print('Left')
		else : print('Right')
		half_yolo = yolo_means_rgb.copy()
		half_yolo = half_yolo[:,highest_median_half*320:320*(highest_median_half+1)]
		cv2.imshow("Half_depth", half_yolo); cv2.waitKey(1)
	except Exception as err : print(err)

	try :
		# Split the image into columns
		columns = np.hsplit(half_yolo, 20)
		#list(filter(lambda x : x != 'something@something.com', emails))
		# Calculate the median pixel value for each column
		medians = [np.mean(col) for col in columns]
		# Find the column with the highest median pixel value
		highest_median_col = np.argmax(medians)

		# Get the region of the image corresponding to the column with the highest median pixel value
		region = yolo_means_rgb.copy()
		xmin = highest_median_half*320 + highest_median_col*half_yolo.shape[1]//20
		xmax = highest_median_half*320 + half_yolo.shape[1]//20*(highest_median_col+1)
		ymin = 0
		ymax = 480
		Xc = int(((xmax-xmin)/2)+xmin) # Center of the object's bounding box
		Yc = int(((ymax-ymin)/2)+ymin)
		region[:, xmin:xmax] = (0,255,0)

		counting_points += 1 
		pointsX.append(Xc)
		pointsY.append(Yc)

		if counting_points > 30 :
			Xc_moy = int(np.mean(pointsX))
			Yc_moy = int(np.mean(pointsY))

			pointsX = []
			pointsY = []
			counting_points = 0

		region = cv2.circle(region, (Xc_moy,Yc_moy), radius=10, color=(255, 0, 0), thickness=-1)

		process_dir([Xc_moy,Yc_moy])
	except : print("Err : spliting")

	#cv2.imshow("Cam", cv_cam_image); cv2.moveWindow("Cam", 0, 0); cv2.waitKey(1)
	#cv2.imshow("Depth", cv_dep_rgb); cv2.moveWindow("Depth", 640, 0); cv2.waitKey(1)
	#cv2.imshow("Dilated_depth", dep_dilated); cv2.waitKey(1)
	#cv2.imshow("Original_depth", cv_dep_image); cv2.waitKey(1)
	#cv2.imshow("New_depth", yolo_means_rgb); cv2.waitKey(1)
	cv2.imshow("Region_depth", region); cv2.waitKey(1)

def process_yolo_img(yolo_image):
	""" 
		Processes the detection image output from darknet_ros' yolov2/3/7
	"""
	global cv_yolo_image
	YOLO_IMG = yolo_image
	cv_yolo_image = bridge.imgmsg_to_cv2(YOLO_IMG, desired_encoding='passthrough')
	
	cv2.imshow("Yolo", cv_yolo_image); cv2.waitKey(1)

def process_dir(direction):
	""" 
		Processes the Trajectory point given as a Twist
	"""
	try :
		Pose_obj = Twist()
		Pose_obj.linear.x = direction[0]
		Pose_obj.linear.y = direction[1]
		Pose_obj.linear.z = cv_dep_image[direction[1],direction[0]]

		Traj_publisher.publish(Pose_obj)
	except : print('No object', direction)

if __name__ == '__main__':
	rospy.init_node('yolo_centroids', anonymous = True)

	global bridge
	global YOLO_IMG
	global cv_yolo_image
	global CAM_IMG 
	global DEP_IMG
	global cv_dep_image
	global OBJs

	global counting_points
	global pointsX
	global pointsY
	global Xc_moy
	global Yc_moy

	Xc_moy,Yc_moy = 320,240
	pointsX = []
	pointsY = []
	counting_points = 0

	bridge = CvBridge()

	Traj_publisher = rospy.Publisher('trajectoire', Twist, queue_size = 1)
	random_pub = rospy.Publisher('sidewalk_random', String, queue_size = 1)
	get_subs()
	
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()