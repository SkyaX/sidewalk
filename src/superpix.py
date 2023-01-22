#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
from skimage.segmentation import slic
from skimage.segmentation import mark_boundaries

from sensor_msgs.msg import Image
from std_msgs.msg import String

depth = 0

def mask_image(image, mask):
    new_Img = image.copy()
    for i, line in enumerate(image):
        for j, pixels in enumerate(line):
            new_Img[i,j] = image[i,j] * mask[i,j]
    return new_Img

def callback(image):
    global depth
    global depth_image
    global cv_dep_image
    global croped_image
    
    depth_image = image

    cv_dep_image = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')

    croped_image = cv_dep_image.copy()


def croped_image_pub():
    pub = rospy.Publisher('depth_squares', Image, queue_size = 1)
    return pub

def depth_image_list():
    rospy.Subscriber('/camera/depth/image_raw', Image,  callback)

if __name__ == '__main__':
    global bridge
    global croped_image
    
    bridge = CvBridge()
    
    croped_image = []
    depth_image_list()

    rospy.init_node('depth_slic', anonymous = True)
    
    while(len(croped_image) == 0):
        pass

    
    ci_pub = croped_image_pub()
    
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        depth_image_cp = croped_image.copy()
        depth_image_cp = cv2.normalize(depth_image_cp, None, alpha = 0, beta = 255, norm_type = cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        segments_slic = slic(depth_image_cp, n_segments=200, compactness=0.1, enforce_connectivity = True)
        
        for classe in np.unique(segments_slic):
            depth_mean = np.mean(depth_image_cp[segments_slic == classe])
            for pxl in depth_image_cp[segments_slic == classe]:
                pxl = depth_mean
        image_message = bridge.cv2_to_imgmsg(depth_image_cp, encoding = 'passthrough')
        ci_pub.publish(image_message)
        rate.sleep()
        cv2.imshow("SLIC", mark_boundaries(depth_image_cp, segments_slic))
        cv2.waitKey(1)

