#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist
import time
import sys
import pyttsx3

def textTospeach(text): #allows to convert string to audio feedback
    engine = pyttsx3.init ()
    engine.say(text)
    engine.runAndWait()
    del engine


def speak(po): 

    go_left = "please move your body to the left until you hear go forward"
    go_right = "please move your body to the right until you hear go forward"
    go_forward = "you can walk in the direction you are in"
    stop="STOP"

    print("z : %f \n", po.linear.z)
    print("x : %f \n", po.linear.x)
    global timestamp 
    print(time.time()- timestamp)

    tps_pause = dict_pause[prev_msg]


    if po.linear.x in range(190,420)  and (time.time()-timestamp) > tps_pause :
        if po.linear.z<1000:
            timestamp = time.time()
            textTospeach(stop)
            prev_msg = 'stop'
        else: 
            timestamp = time.time()
            textTospeach(go_forward)
            prev_msg = 'forward'

    if po.linear.x not in range(190,420)  and (time.time()-timestamp) > tps_pause :
        if po.linear.x<190:
            timestamp = time.time()
            textTospeach(go_left)
            prev_msg = 'rotate'
        elif po.linear.x>420 :
            timestamp = time.time()
            textTospeach(go_right)
            prev_msg = 'rotate'


def getInfo(): 
    rospy.init_node('getInfo',anonymous=True)
    global timestamp 
    timestamp = 0
    rospy.Subscriber('/trajectoire',Twist,speak,queue_size=2) #modifier avec nos parametres a nous


    while not rospy.is_shutdown():
        rospy.sleep(0.5)


if __name__=='__main__': 

    global dict_pause
    global prev_msg

    prev_msg = 'stop'

    dict_pause = {
        'stop' : 1,
        'forward' : 1,
        'hard' : 1.5,
        'rotate' : 3
    }

    try: 
        getInfo()
    except rospy.ROSInterruptException: 
        pass


