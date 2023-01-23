#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist
import time
import sys

import speake3
#import pyttsx3

def textTospeach(text): #allows to convert string to audio feedback
     #pyttsx3.init ()
    global engine 

    engine.say(text)
    engine.talkback()
    #engine.runAndWait()
    #del engine

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
                timestamp = time.time()
                textTospeach("stop")
                prev_msg = 'stop'
            else: 
                timestamp = time.time()
                textTospeach("go forward")
                prev_msg = 'forward'

        elif X in range(0,239) :
            if X<=96 :
                timestamp = time.time()
                textTospeach("rotate hard left")
                prev_msg = 'hard'
            else :
                timestamp = time.time()
                textTospeach("rotate left")
                prev_msg = 'rotate'
        else:
            if X >=594 :
                timestamp = time.time()
                textTospeach("rotate hard right")
                prev_msg = 'hard'
            else:
                timestamp = time.time()
                textTospeach("rotate right")
                prev_msg = 'rotate'
    


def getInfo(): 
    rospy.init_node('getInfo',anonymous=True)
    global timestamp 
    timestamp = 0
    rospy.Subscriber('/trajectoire',Twist,speak,queue_size=2)

    while not rospy.is_shutdown():
        rospy.sleep(0.5)

if __name__=='__main__': 

    global dict_pause
    global prev_msg
    global engine

    engine = speake3.Speake()
    engine.set('voice', 'en')
    engine.set('speed', '107')
    #engine.set('pitch', '99')

    engine.say("please wait")
    prev_msg = 'wait'
    

    dict_pause = {
        'stop' : 1,
        'forward' : 1,
        'hard':1.5, 
        'rotate' : 2, 
        'wait':5,
    }

    try: 
        getInfo()
    except rospy.ROSInterruptException: 
        pass