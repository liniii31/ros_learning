#!/usr/bin/env python

import sys
import rospy
from ros_learning.srv import *


def call_server():
    rospy.wait_for_service('move_in_circle')
    try:
        add_two_ints = rospy.ServiceProxy('move_in_circle', srv_msg)
        
        #print("________________________________________________________________________________")
        #print("--------------------------------------------------------------------------------")
        #s1 = input("Enter the time for which the base_link will rotate in seconds : ")
        s1 = 30
        add_two_ints(s1)
        
    except rospy.ServiceException as e:
        print("THE END")


if __name__ == "__main__":
	call_server()

