#!/usr/bin/env python3

import rospy 
from darknet_ros_msgs.msg import BoundingBoxes as Bb

from geometry_msgs.msg import Twist

from turtlesim.msg import Pose # a remplacer avec les infos de trajectory 

import sys
import pyttsx3

def call(po): 
    rospy.loginfo("X = %f : Y = %f : Z=%f\n", po.x, po.y, po.theta)

def speak(po): 
    #remplacer les valeurs aves le xmin,xmax,ymin et ymax de notre video 
    #permet de guider la personne
    if (po.x < 1.35) : 
        if (po.y<4.38) :
            pyttsx3.speak("go right and forward")
        elif (po.y > 7.19): 
            pyttsx3.speak("go right and move back ")
        else : 
            pyttsx3.speak('go right')
    elif (po.x > 9.63) : 
        if (po.y<4.38) : 
            pyttsx3.speak("go left and forward")
        elif (po.y > 7.19) : 
            pyttsx3.speak("go left and move back")
        else : 
            pyttsx3.speak("go left")
    elif (po.y > 7.19): 
        pyttsx3.speak('go boack')
    elif (po.y < 4.38): 
        pyttsx3.speak("go forward")




def getInfo(): 
    rospy.init_node('getInfo',anonymous=True)
    pub = rospy.Subscriber('/turtle1/pose',Pose,call,queue_size=2)  #modifier avec nos parametres a nous
    spea=rospy.Subscriber('/turtle1/pose',Pose,speak) #modifier avec nos parametres a nous


    while not rospy.is_shutdown():
        rospy.sleep(0.1)


if __name__=='__main__': 
    try: 
        getInfo()
    except rospy.ROSInterruptException: 
        pass


