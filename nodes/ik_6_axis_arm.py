#!/usr/bin/env python 
# coding: utf-8 
import rospy
from geometry_msgs.msg import Pose
from ros_learning.msg import ik_for_6axis
import tf_conversions
import math
import tf2_ros
from sympy import *


def ik():
    a2 = 0.305000
    d3 = 0.0 
    a3 = 0.010000
    d4 = 0.164500
    #pi = 3.14159
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    i = ik_for_6axis()
    joint_pub = rospy.Publisher('joint_pose', ik_for_6axis, queue_size=1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            
            trans1 = tfBuffer.lookup_transform('base_link', 'tool', rospy.Time())
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        #Finding the first three joint angles i.e. J1,J2,J3
        px = trans1.transform.translation.x
        py = trans1.transform.translation.y
        pz = trans1.transform.translation.z
        (alpha,beta,gamma) = tf_conversions.transformations.euler_from_quaternion([trans1.transform.rotation.x, trans1.transform.rotation.y, trans1.transform.rotation.z, trans1.transform.rotation.w])
        R_x = Matrix([[1,0,0],[0,math.cos(alpha),-math.sin(alpha)],[0,math.sin(alpha),math.cos(alpha)]])
        R_y = Matrix([[math.cos(beta),0,math.sin(beta)],[0,1,0],[-math.sin(beta),0,math.cos(beta)]])
        R_z = Matrix([[math.cos(gamma),-math.sin(gamma),0],[math.sin(gamma),math.cos(gamma),0],[0,0,1]])
        R = R_x*R_y*R_z
        #joint angle 1
        i.theta1 = math.atan2(py,px)-math.atan2(d3,math.sqrt(abs(Pow(px,2)+Pow(py,2)-Pow(d3,2))))
        theta1 = i.theta1
        K = (Pow(px,2)+Pow(py,2)+Pow(pz,2)-Pow(a2,2)-Pow(a3,2)-Pow(d3,2)-Pow(d4,2))/(2*a2)
        #joint angle 3
        i.theta3 = math.atan2(a3,d4)-math.atan2(K,math.sqrt(abs(Pow(a3,2)+Pow(d4,2)-Pow(K,2))))
        theta3 = i.theta3
        theta23 = math.atan2((-a3-a2*math.cos(theta3))*pz-(math.cos(theta1)*px+math.sin(theta1)*py)*(d4-a2*math.sin(theta3)),(a2*math.sin(theta3)-d4)*pz-(a3+a2*math.cos(theta3))*(math.cos(theta1)*px+math.sin(theta1)*py))
        #joint angle 2
        i.theta2 = theta23-i.theta3
        theta2 = i.theta2
        #joint angle 4
        i.theta4 = math.atan2(-R[0,2]*math.sin(theta1)+R[1,2]*math.cos(theta1),-R[0,2]*math.cos(theta1)*math.cos(theta23)-R[1,2]*math.sin(theta1)*math.cos(theta23)+R[2,2]*math.sin(theta23))
        theta4 = i.theta4
        s5 = -(R[0,2]*(math.cos(theta1)*math.cos(theta23)*math.cos(theta4)+math.sin(theta1)*math.sin(theta4))+R[1,2]*(math.sin(theta1)*math.cos(theta23)*math.cos(theta4)-math.cos(theta1)*math.sin(theta4))-R[2,2]*(math.sin(theta23)*math.cos(theta4)))
        c5 = R[0,2]*(-math.cos(theta1)*math.sin(theta23))+R[1,2]*(-math.sin(theta1)*math.sin(theta23))+R[2,2]*(-math.cos(theta23))
        #joint angle 5
        i.theta5 = math.atan2(s5,c5)
        theta5 = i.theta5
        s6 = -R[0,0]*(math.cos(theta1)*math.cos(theta23)*math.sin(theta4)-math.sin(theta1)*math.cos(theta4))-R[1,0]*(math.sin(theta1)*math.cos(theta23)*math.sin(theta4)+math.cos(theta1)*math.cos(theta4))+R[2,0]*(math.sin(theta23)*math.sin(theta4))
        c6 = R[0,0]*((math.cos(theta1)*math.cos(theta23)*math.cos(theta4)+math.sin(theta1)*math.sin(theta4))*math.cos(theta5)-math.cos(theta1)*math.sin(theta23)*math.sin(theta5))+R[1,0]*((math.sin(theta1)*math.cos(theta23)*math.cos(theta4)-math.cos(theta1)*math.sin(theta4))*math.cos(theta5)-math.sin(theta1)*math.sin(theta23)*math.sin(theta5))-R[2,0]*(math.sin(theta23)*math.cos(theta4)*math.cos(theta5)+math.cos(theta23)*math.sin(theta5))
        #joint angle 6
        i.theta6 = math.atan2(s6,c6)
        #publishing the joint angles
        joint_pub.publish(i) 




if __name__ == '__main__':
    rospy.init_node('pose_listener', anonymous=True)
    try:
        ik()
    except rospy.ROSInterruptException:
        pass