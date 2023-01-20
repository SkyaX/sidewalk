#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2

from sensor_msgs.msg import Image
from std_msgs.msg import String

def mask_image(image, mask):
    new_Img = image.copy()
    for i, line in enumerate(image):
        for j, pixels in enumerate(line):
            new_Img[i,j] = image[i,j] * mask[i,j]
    return new_Img

def callback(image):
    global depth
    global depth_image
    global croped_image

    depth = 0
    depth_image = image

    cv_dep_image = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

    croped_image = cv_dep_image.copy() #[240:288,:].copy()
    for i, line in enumerate(croped_image):
        for j, pxl in enumerate(line):
            if np.isnan(pxl):
                croped_image[i,j] = 0
    for i in range(20):
        square = croped_image[:,i*32:(i+1)*32]
        if(len(square[square != 0])==0):
            d_mean = 0
        else :
            d_mean = np.mean(square[square != 0])
        for j, line  in enumerate(square):
            for k, pxl in enumerate(line):
                square[j,k] = d_mean 

    croped_image = cv2.normalize(croped_image, croped_image, alpha = 0, beta = 255, norm_type = cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    cv2.imshow("Depth", croped_image)

    cv2.waitKey(1)

def croped_image_pub():
    pub = rospy.Publisher('depth', Image, queue_size = 1)
    return pub

def depth_image_list():
    rospy.Subscriber('/camera/depth/image_raw', Image,  callback)
    #rospy.Subscriber('/camera/depth_registered/sw_registered/image_rect', Image,  callback)

if __name__ == '__main__':
    global bridge
    global croped_image
    
    bridge = CvBridge()
    
    croped_image = []
    depth_image_list()

    rospy.init_node('depth_cropped', anonymous = True)
    
    while(len(croped_image) == 0):
        pass

    
    ci_pub = croped_image_pub()
    
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        image_message = bridge.cv2_to_imgmsg(croped_image, encoding = 'passthrough')
        ci_pub.publish(image_message)
        rate.sleep()

