#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
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

	DEP_IMG = depth_image
	CAM_IMG = cam_image
	cv_cam_image = bridge.imgmsg_to_cv2(CAM_IMG, desired_encoding='passthrough')
	cv_dep_image = bridge.imgmsg_to_cv2(DEP_IMG, desired_encoding='passthrough')

	kernel = np.ones((7,7), np.uint8)
	dep_dilated = cv2.dilate(cv_dep_image, kernel)

	try :
		yolo_means = dep_dilated.copy()
		for object in OBJs :
			object_img = yolo_means[object.ymin:object.ymax,object.xmin:object.xmax]
			object.median_depth = np.median(np.reshape(object_img, (1,-1)))
			# random_pub.publish(f"{object.obj_type} : distance = {median_depth}\n")

		for object in np.sort(OBJs)[::-1] : 
			print(f"{object.obj_type} : {object.median_depth}")
			yolo_means[object.ymin:object.ymax,object.xmin:object.xmax] = object.median_depth

	except : print("Err : try yolo_means")

	cv_dep_norm = cv2.normalize(dep_dilated, None, alpha = 0, beta = 255, norm_type = cv2.NORM_MINMAX, dtype=cv2.CV_8U)
	cv_dep_rgb = cv2.cvtColor(cv_dep_norm, cv2.COLOR_GRAY2RGB, 0);

	yolo_means_norm = cv2.normalize(yolo_means, None, alpha = 0, beta = 255, norm_type = cv2.NORM_MINMAX, dtype=cv2.CV_8U)
	yolo_means_rgb = cv2.cvtColor(yolo_means_norm, cv2.COLOR_GRAY2RGB, 0);
	
	"""MAX = np.max(np.reshape(cv_dep_image, (1,-1)))
	HIST = cv2.calcHist(cv_dep_image,[0],None,[MAX+1],[0,MAX])
	plt.plot(HIST); plt.show() # plt.savefig("hist.png")
	plt.imshow(cv_dep_image); plt.show()"""

	try : # goes throught the detected objects and displays their centroid
		for object in OBJs :
			coord = (object.Xc, object.Yc)
			cv_cam_image = cv2.circle(cv_cam_image, coord, radius=10, color=(0, 255, 0), thickness=-1)
			cv_dep_rgb = cv2.circle(cv_dep_rgb, coord, radius=10, color=(0, 255, 0), thickness=-1)

	except : print("Err : try green circles")

	#cv2.imshow("Cam", cv_cam_image); cv2.moveWindow("Cam", 0, 0); cv2.waitKey(1)
	#cv2.imshow("Depth", cv_dep_rgb); cv2.moveWindow("Depth", 640, 0); cv2.waitKey(1)
	cv2.imshow("Dilated_depth", dep_dilated); cv2.moveWindow("Dilated_depth", 640, 0); cv2.waitKey(1)
	cv2.imshow("Original_depth", cv_dep_image); cv2.moveWindow("Original_depth", 1280, 0); cv2.waitKey(1)
	cv2.imshow("New_depth", yolo_means_rgb); cv2.moveWindow("New_depth", 0, 480); cv2.waitKey(1)

def process_yolo_img(yolo_image):
	""" 
		Processes the detection image output from darknet_ros' yolov2/3/7
	"""
	global cv_yolo_image
	YOLO_IMG = yolo_image
	cv_yolo_image = bridge.imgmsg_to_cv2(YOLO_IMG, desired_encoding='passthrough')
	
	cv2.imshow("Yolo", cv_yolo_image); cv2.moveWindow("Yolo", 640, 480); cv2.waitKey(1)

def process_dir(direction : Twist):
	""" 
		Processes the Trajectory point given as a Twist
	"""
	try :
		Pose_obj = Twist()
		Pose_obj.linear.x = direction.Xc
		Pose_obj.linear.y = direction.Yc
		Pose_obj.linear.z = cv_dep_image[direction.Yc,direction.Xc]

		Traj_publisher.publish(Pose_obj)
	except : print('No object')

if __name__ == '__main__':
	rospy.init_node('yolo_centroids', anonymous = True)

	global bridge
	global YOLO_IMG
	global cv_yolo_image
	global CAM_IMG 
	global DEP_IMG
	global cv_dep_image
	global OBJs

	bridge = CvBridge()

	Traj_publisher = rospy.Publisher('trajectoire', Twist, queue_size = 1)
	random_pub = rospy.Publisher('sidewalk_random', String, queue_size = 1)
	get_subs()
	
	rospy.spin()
