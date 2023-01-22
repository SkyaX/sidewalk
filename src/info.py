#!/usr/bin/env python3

import rospy 

#from sidewalk.msg import Trajectoire 

from geometry_msgs.msg import Twist

# from turtlesim.msg import Pose # a remplacer avec les infos de trajectory 

import sys
import pyttsx3

# voice = pyttsx3.init ()

# """RATE """
# rate = voice.getProperty('rate')
# voice.setProperty('rate',180)

# """VOLUME"""
# volume = voice.getProperty('volume')
# voice.setProperty('volume',0.5)

# """VOICE = woman """
# voices = voice.getProperty('voices') 
# voice.setProperty('voice', voices[0].id)



def call(po): 
    #rospy.loginfo("X = %f : Y = %f : Z=%f\n", po.linear.x, po.linear.y, po.linear.z)
    print(po.linear.x)


def speak(po): 
    #remplacer les valeurs aves le xmin,xmax,ymin et ymax de notre video 
    #permet de guider la personne

    print(po.linear.x)

    if po.linear.x not in range(224,416) or po.z<2: 
        if po.linear.z<2 : 
            pyttsx3.speak("STOP")
        if po.linear.x<224: 
            pyttsx3.speak("please move your body to the left until you hear go forward")
        elif po.linear.x>416 :
            pyttsx3.speak("please move your body right until you hear go forward")
    else : 
        pyttsx3.speak("you can walk in the direction you are in")

    # if (po.x < 1.35) : 
    #     if (po.y<4.38) :
    #         pyttsx3.speak("go right and up")
    #     elif (po.y > 7.19): 
    #         pyttsx3.speak("go right and down")
    #     else : 
    #         pyttsx3.speak('go right')
    # elif (po.x > 9.63) : 
    #     if (po.y<4.38) : 
    #         pyttsx3.speak("go left and up")
    #     elif (po.y > 7.19) : cv2.imshow("Cam", cv_cam_image); cv2.waitKey(1)
	# cv2
    #         pyttsx3.speak("go left and down")
    #     else : 
    #         pyttsx3.speak("go left")
    # elif (po.y > 7.19): 
    #     pyttsx3.speak('go down')
    # elif (po.y < 4.38): 
    #     pyttsx3.speak("go up")

    #voice.runAndWait()


def getInfo(): 
    rospy.init_node('getInfo',anonymous=True)
    #rospy.Subscriber('/trajectoire',Twist,call,queue_size=2)  #modifier avec nos parametres a nous
    rospy.Subscriber('/trajectoire',Twist,speak) #modifier avec nos parametres a nous


    while not rospy.is_shutdown():
        rospy.sleep(0.5)


if __name__=='__main__': 
    try: 
        getInfo()
    except rospy.ROSInterruptException: 
        pass


