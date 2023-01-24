#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes as BBs
from darknet_ros_msgs.msg import BoundingBox as BB

class OBJ :
    def __init__(self, bb) :
        self.Bbox = bb

        self.obj_type = bb.Class

        self.id = bb.id

        self.xmax = bb.xmax
        self.xmin = bb.xmin
        self.ymax = bb.ymax
        self.ymin = bb.ymin

        self.Xc = int(((self.xmax-self.xmin)/2)+self.xmin)
        self.Yc = int(((self.ymax-self.ymin)/2)+self.ymin)

def get_subs():
    rospy.Subscriber('/darknet_ros/bounding_boxes', BBs,  process_BBs)
    rospy.Subscriber('/depth_squares', Image, process_depth_squares)
    cam_img_sub = rospy.Subscriber('/camera/color/image_raw', Image, process_cam_image)

def process_cam_image(img) :
    global cv_cam_image
    cv_cam_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')

def process_BBs(BBS):
    #print("\n---------------------------------------")
    global OBJs
    global i_mean_changed
    global i_mean
    global n_means

    OBJs = [ OBJ(bb) for bb in BBS.bounding_boxes if bb.probability>0.30 ] 
    i_mean_changed = True
    i_mean = i_mean + 1
    if (i_mean == n_means):
        i_mean = 0

def process_depth_squares(image):
    global bridge
    global depth_sq_img

    depth_sq_img = bridge.imgmsg_to_cv2(image, desired_encoding = 'passthrough')

def process_dir(direction):
    global i_le
    global nb_div
    global pxl_div
    global depth_sq_img
    global n_means
    global i_les
    global i_mean
    global i_mean_changed

    free_sp = np.argwhere(direction < 1)[:,0]
    if (len(free_sp) == 0): # No space to move
        if(np.sum(direction[:(nb_div//2)]) > np.sum(direction[(nb_div//2):])):
            i_le = 0
        else :
            i_le = 639
    else :
        i_longest_streak = 0
        longest_streak = 1
        streak = 1
        for i in range(len(free_sp) -1):
            if free_sp[i+1] != free_sp[i] + 1 or i+2 == len(free_sp):
                if (streak > 2 and np.abs(nb_div//2 - (i - streak//2 + 1)) < np.abs(nb_div//2 - (i_longest_streak - longest_streak//2 + 1))):
                    longest_streak = streak
                    i_longest_streak = i - longest_streak + 1
                streak = 1
            else:
                streak+=1
        i_le = (free_sp[i_longest_streak] + longest_streak/2)*pxl_div
    try:
        if (i_mean_changed):
            i_les[i_mean] = i_le
            i_mean_change = False
    except : pass
    i_le = np.mean(i_les)
    dir_msg = Twist()
    dir_msg.linear.x = i_le
    dir_msg.linear.y = 480//2
    try : 
        cv2.imshow("Test", depth_sq_img); cv2.waitKey(1)
        dir_msg.linear.z = depth_sq_img[int(480//2),int(i_le)]
        if (dir_msg.linear.z == 0):
            dir_msg.linear.z = 4000
    except : print("Pas d'image")
    dir_msg.angular.z = 0
    dir_msg.angular.x = 0
    dir_msg.angular.y = 0
    
    return(dir_msg)

def score_computing() :

    global cv_cam_image
    global depth_sq_img

    score = np.zeros(nb_div)

    for object in OBJs:
        try :
            d_cent = depth_sq_img[object.Yc,object.Xc]

            if (d_cent < 3000 and d_cent != 0):
                for i in range(len(score)):
                    thb = i*(pxl_div)
                    thh = (i+1)*pxl_div
                    score_io = pxl_div
                    if (object.xmin > thb):
                        score_io -= (object.xmin - thb)
                    if (object.xmax < thh):
                        score_io -= (thh - object.xmax)
                    if (object.xmin > thh or thb > object.xmax):
                        score_io = 0
                    score[i] += score_io
        except : print("Pas d'objet")
    pose_msg = process_dir(score)
    Traj_publisher.publish(pose_msg)
    cv_im = cv_cam_image.copy()

    try :
        cv_im = cv2.circle(cv_im, (int(i_le), int(210)), radius=10, color=(0, 255, 0), thickness=-1)
        cv2.imshow("Cam", cv_im)
        cv2.waitKey(1)
    except : print('No img to show')

if __name__ == '__main__':
    rospy.init_node('trajectoire_matthieu', anonymous = True)
    
    global OBJs
    global bridge
    global depth_sq_img
    global cv_cam_image
    global i_le
    global nb_div
    global pxl_div
    global i_les
    global n_means

    global i_means

    n_means = 7
    i_mean = 0

    i_dir = 0

    i_les = np.zeros(n_means)

    nb_div = 20
    pxl_div = 640//nb_div

    i_le = -1
    cv_cam_image = []
    OBJs = []
    depth_sq_img = []
    
    bridge = CvBridge()

    get_subs()
    
    Traj_publisher = rospy.Publisher('/trajectoire', Twist, queue_size=2)

    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        score_computing()
        rate.sleep()

