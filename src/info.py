#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import time
import sys

from cv_bridge import CvBridge
import cv2

import speake3
#import pyttsx3

def textTospeach(text): #allows to convert string to audio feedback
    global engine 
    engine.say(text)
    engine.talkback()

def speak(po): 
    global prev_msg

    X = po.linear.x
    Y = po.linear.y
    Z = po.linear.z

    print("z : %f \n", Z)
    print("x : %f \n", X)
    global timestamp 
    print(time.time()- timestamp)

    tps_pause = dict_pause[prev_msg]

    if (time.time()-timestamp) > tps_pause :
        if X in range(240,440): 
            if Z<1000:
                if prev_msg != 'stop' : textTospeach("stop, slowly go backward")
                else : textTospeach("keep slowly going backward")
                prev_msg = 'stop'
            else: 
                textTospeach("go forward")
                prev_msg = 'forward'

        elif X in range(0,239) :
            if X<=96 :
                if prev_msg != 'rotate' : textTospeach("stop, slightly rotate left")
                else : textTospeach("keep slightly rotating left")
                prev_msg = 'rotate'
            else :
                textTospeach("slow forward light left")
                prev_msg = 'light'
        else:
            if X >=594 :
                if prev_msg != 'rotate' : textTospeach("stop, slightly rotate right")
                else : textTospeach("keep slightly rotating right")
                prev_msg = 'rotate'
            else:
                textTospeach("slow forward light right")
                prev_msg = 'light'
        timestamp = time.time()

def process_img_traj(img) :
    region = bridge.imgmsg_to_cv2(img)

    cv2.imshow("Region_info", region); cv2.waitKey(1)

def getInfo(): 
    rospy.init_node('getInfo',anonymous=True)
    global timestamp 
    timestamp = 0
    rospy.Subscriber('/trajectoire',Twist,speak,queue_size=5)
    rospy.Subscriber('/img_trajectoire', Image, process_img_traj)

    while not rospy.is_shutdown():
        rospy.sleep(0.5)

if __name__=='__main__': 

    global dict_pause
    global prev_msg
    global engine
    global bridge 

    bridge = CvBridge()

    engine = speake3.Speake()
    engine.set('voice', 'en')
    engine.set('speed', '107')
    #engine.set('pitch', '99')

    engine.say("please wait")
    prev_msg = 'wait'

    dict_pause = {
        'stop' : 0.5,
        'forward' : 0.5,
        'light':1, 
        'rotate' : 1.5, 
        'wait':5,
    }

    try: 
        getInfo()
    except rospy.ROSInterruptException: 
        pass