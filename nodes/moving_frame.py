#!/usr/bin/env python  

import roslib
roslib.load_manifest('ros_learning')
from ros_learning.srv import *
import rospy
import tf
import time
import math


def handle_frame(req):
      
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    br.sendTransform((3.0, 3.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "base_link",
                         "map")
    print("..................STARTED................")
    t_end = time.time() + req.s1
    while time.time() < t_end:
        t = rospy.Time.now().to_sec() * math.pi
        br.sendTransform((3.0 * math.sin(t), 3.0 * math.cos(t), 0.0),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"base_link","map")
        rate.sleep()
       
    

def revolve():
    s = rospy.Service('move_in_circle', srv_msg, handle_frame)
    
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('moving_frame')
    revolve()
    

