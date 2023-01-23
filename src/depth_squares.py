#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2

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
    rospy.Subscriber('/camera/depth/image_raw', Image,  callback, queue_size = 1)

if __name__ == '__main__':
    global bridge
    global croped_image
    
    bridge = CvBridge()
    
    croped_image = []
    depth_image_list()

    rospy.init_node('depth_square', anonymous = True)
    
    while(len(croped_image) == 0):
        pass

    
    ci_pub = croped_image_pub()
    
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        croped_image_cp = croped_image.copy()    
        
        croped_image_cp[np.all(croped_image_cp == float('nan'), axis=-1)] = 0
        
        for l in range(40):
            for i in range(40):
                square = croped_image_cp[l*12:(l+1)*12:,i*16:(i+1)*16]
                if(len(square[square != 0])==0):
                    d_mean = 0
                else :
                    d_mean = np.mean(square[square != 0])
                for j, line  in enumerate(square):
                    for k, pxl in enumerate(line):
                        square[j,k] = d_mean
        croped_image_cp = cv2.normalize(croped_image_cp, None, alpha = 0, beta = 255, norm_type = cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        image_message = bridge.cv2_to_imgmsg(croped_image_cp, encoding = 'passthrough')
        ci_pub.publish(image_message)
        rate.sleep()
        cv2.imshow("Depth_squares", croped_image_cp); cv2.waitKey(1)

