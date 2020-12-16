#!/usr/bin/env python  

import rospy

# Because of transformations
import tf_conversions
import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.msg
from ros_learning.srv import *



def handle_turtle_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    br.sendTransform(t)


def handle(req):
    rospy.Subscriber('/turtle1/pose',
                     turtlesim.msg.Pose,
                     handle_turtle_pose)
    rospy.spin()  
           
    

def revolve():
    s = rospy.Service('move_in_circle', srv_msg, handle)
    
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('moving_frame')
    revolve()
    

