#!/usr/bin/env python

# license removed for brevity
import rospy
import math
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from ros_learning.srv import *
flag = True

def start_service(req):
    global omega,radius,rate 

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    
    # periodic rate: 
    r = rospy.Rate(rate)
    # timer counter [second]
    tick = 0
    inc = 1. /rate      #increment of tick for different values of periodic rate
    
    global flag 
    flag = True
    while(flag):
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        # This is target angle around z axis
        theta = omega*tick
        t.transform.translation.x = math.sin(theta)*radius
        t.transform.translation.y = math.cos(theta)*radius
        t.transform.translation.z = 0
        # The orientation should be set as you did
        # set it negative because we want to make x-xaxis to frontal direction
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, -theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        # Next period
        tick = tick + inc
        r.sleep()

    return startResponse()


def stop_service(req):
    global flag
    flag = False
    return stopResponse()


def publisher(omega,radius,rate):
    s1= rospy.Service('start',start,start_service)
    s2= rospy.Service('stop',stop,stop_service)
    rospy.spin()

if __name__ == '__main__':

    rospy.init_node('publisher', anonymous=True)
    global omega,radius,rate
    omega = rospy.get_param('~omega')       # angular velocity [rad/s]
    radius = rospy.get_param('~radius')
    rate = rospy.get_param('~rate')
    try:
        
        publisher(omega,radius,rate)
    except rospy.ROSInterruptException:
        pass
