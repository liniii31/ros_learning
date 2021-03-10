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
    a1, a2, a3, a4, a5, a6 = 0.181500, 0.163500, 0.305000, 0.164500, 0.135500, 0.070000
    b4 = 0.010000
    pi = 3.14159
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
        x = trans1.transform.translation.x
        y = trans1.transform.translation.y
        z = trans1.transform.translation.z
        q0 = trans1.transform.rotation.x
        q1 = trans1.transform.rotation.y
        q2 = trans1.transform.rotation.z
        q3 = trans1.transform.rotation.w
        (alpha,beta,gamma) = tf_conversions.transformations.euler_from_quaternion([trans1.transform.rotation.x, trans1.transform.rotation.y, trans1.transform.rotation.z, trans1.transform.rotation.w])
        R_x = Matrix([[1,0,0],[0,math.cos(alpha),-math.sin(alpha)],[0,math.sin(alpha),math.cos(alpha)]])
        R_y = Matrix([[math.cos(beta),0,math.sin(beta)],[0,1,0],[-math.sin(beta),0,math.cos(beta)]])
        R_z = Matrix([[math.cos(gamma),-math.sin(gamma),0],[math.sin(gamma),math.cos(gamma),0],[0,0,1]])
        R = R_x*R_y*R_z 
        Px = x - ((a6)*R[0,2])
        Py = y - ((a6)*R[1,2])
        Pz = z - ((a6)*R[2,2])
        i.J1 = float(format(math.atan2(Py,Px),'.3f'))
        Pxy = math.sqrt(math.pow(Px,2)+math.pow(Py,2))
        l1 = Pz-(a1+a2)
        phi2 = math.atan2(l1,Pxy)
        l2 = math.sqrt(math.pow(l1,2)+math.pow(Pxy,2))
        l3 =  math.sqrt(math.pow(b4,2)+math.pow((a4+a5),2))
        phi4 = math.atan2(b4,(a4+a5))
        C_phi3 = (math.pow(l3,2)+math.pow(a3,2)-math.pow(l2,2))/(2*l3*a3)
        S_phi3 = math.sqrt(abs(1-math.pow(C_phi3,2)))
        phi3 = math.atan2(S_phi3,C_phi3)
        C_phi1 = (math.pow(a3,2)+math.pow(l2,2)-math.pow(l3,2))/(2*l2*a3)
        S_phi1 = math.sqrt(abs(1-math.pow(C_phi1,2)))
        phi1 = math.atan2(S_phi1,C_phi1)  
        i.J2 = float(format((pi/2)-(phi2+phi1),'.3f'))
        i.J3 = float(format(pi-(phi4+phi3),'.3f'))          
        #When First three Joint anles are found,
        #We can start finding the last Three Joint angles
        R1 = Matrix([[math.cos(i.J1),-math.sin(i.J1),0],[math.sin(i.J1),math.cos(i.J1),0],[0,0,1]])
        R2 = Matrix([[math.cos(i.J2),0,math.sin(i.J2)],[0,1,0],[-math.sin(i.J2),0,math.cos(i.J2)]])
        R3 = Matrix([[math.cos(i.J3),0,math.sin(i.J3)],[0,1,0],[-math.sin(i.J3),0,math.cos(i.J3)]])
        R4 = Matrix([[math.cos(0),-math.sin(0),0],[math.sin(0),math.cos(0),0],[0,0,1]])
        R03 = R1*R2*R3
        R03_Inverse = Inverse(R03)
        #phi = math.atan2((math.sin(alpha)*math.sin(beta)*math.cos(gamma))+(math.cos(alpha)*math.sin(gamma)),math.cos(beta)*math.cos(gamma))
        #R06 = Matrix([[math.cos(phi),-math.sin(phi),0],[math.sin(phi),math.cos(phi),0],[0,0,1]])
        R06 = Matrix([[2*(Pow(q0,2)+Pow(q1,2))-1,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2)],[2*(q1*q2+q0*q3),2*(Pow(q0,2)+Pow(q2,2))-1,2*(q2*q3-q0*q1)],[2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),2*(Pow(q0,2)+Pow(q3,2))-1]])
        R36 = R03_Inverse*R06
        R04 = R1*R2*R3*R4
        i.J4 = float(format(math.atan2(R36[1,2],-R36[0,2]),'.2f'))
        i.J5 = float(format(math.atan2(math.sqrt(abs(1-R36[2,2])),R36[2,2]),'.2f'))
        i.J6 = float(format(math.atan2(-R36[2,1],-R36[2,0]),'.2f'))
        joint_pub.publish(i)           #Publish joint's pose




if __name__ == '__main__':
    rospy.init_node('pose_listener', anonymous=True)
    try:
        ik()
    except rospy.ROSInterruptException:
        pass