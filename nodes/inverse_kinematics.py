#!/usr/bin/env python 
# coding: utf-8 
import rospy
from geometry_msgs.msg import Pose
from ros_learning.msg import ik
import tf_conversions
import math
import tf2_ros

def listener():
    a1, a2 = 0.25, 0.25 
    b1, b2, b4 = 0.059, 0.16, 0
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    p = Pose()
    i = ik()
    tool_pub = rospy.Publisher('tool_pose', Pose, queue_size=1)
    joint_pub = rospy.Publisher('joint_pose', ik, queue_size=1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Listening the pose of the tool frame , represented by Quaternions
            trans = tfBuffer.lookup_transform('base_link', 'tool', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        p.position.x = trans.transform.translation.x
        p.position.y = trans.transform.translation.y
        p.position.z = trans.transform.translation.z
        p.orientation.x = trans.transform.rotation.x
        p.orientation.y = trans.transform.rotation.y
        p.orientation.z = trans.transform.rotation.z
        p.orientation.w = trans.transform.rotation.w
        
        # tool_frame's position : px,py,pz
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        # tool_frame's orientation : alpha = yaw, beta = pitch, gamma = roll
        (gamma, beta, alpha) = tf_conversions.transformations.euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        
        # rotation matrix of tool_frame with respect to the base frame interms of alpha = yaw, beta = pitch, gamma = roll :
        #
        # ⎡1.0⋅cos(α)⋅cos(β)  -1.0⋅sin(α)⋅cos(γ) + sin(β)⋅sin(γ)⋅cos(α)  1.0⋅sin(α)⋅sin(γ) + sin(β)⋅cos(α)⋅cos(γ) ⎤
        # ⎢                                                                                                       |
        # ⎢1.0⋅sin(α)⋅cos(β)  sin(α)⋅sin(β)⋅sin(γ) + 1.0⋅cos(α)⋅cos(γ)   sin(α)⋅sin(β)⋅cos(γ) - 1.0⋅sin(γ)⋅cos(α) ⎥
        # ⎢                                                                                                       |
        # ⎣   -1.0⋅sin(β)                 1.0⋅sin(γ)⋅cos(β)                         1.0⋅cos(β)⋅cos(γ)             ⎦
        #
        #
        # Transformation matrix of tool_frame with respect to the base frame obtained from T = T1 T2 T3 T4 :
        #
        # ⎡ C(124)  -S(124)  0    a1C1 + a2C12      ⎤
        # ⎢                                         ⎥
        # ⎢ S(124)  C(124)   0    a1S1 + a2S12      ⎥
        # ⎢                                         ⎥
        # ⎢ 0       0        1    b1 + b2 + b3 - b4 ⎥
        # ⎢                                         ⎥
        # ⎣ 0       0        0    1                 ⎦
        #
        #Transformation matrix obtain from the pose of tool_frame with respect to base frame
        #
        # ⎡ C(phi)  -S(phi)  0    x      ⎤
        # ⎢                              ⎥
        # ⎢ S(phi)  C(phi)   0    y      ⎥
        # ⎢                              ⎥
        # ⎢ 0       0        1    z      ⎥
        # ⎢                              ⎥
        # ⎣ 0       0        0    1      ⎦

        phi = math.atan2(math.sin(alpha)*math.cos(beta),math.cos(alpha)*math.cos(beta))
        theta123 = math.atan2(math.sin(phi),math.cos(phi))
        #finding theta2 
        cos_theta2 = (math.pow(x,2)+math.pow(y,2)-math.pow(a1,2)-math.pow(a2,2))/(2*a1*a2)  
        sin_theta2 = math.sqrt(abs(1-math.pow(cos_theta2,2)))
        i.theta2 = math.atan2(sin_theta2,cos_theta2)
        #finding theta1
        cos_theta1 = (((a1+(a2*cos_theta2))*x)+(a2*sin_theta2*y))/((math.pow(a1+(a2*cos_theta2),2))-(math.pow(a2*sin_theta2,2)))
        sin_theta1 = (((a1+(a2*cos_theta2))*y)-(a2*sin_theta2*x))/((math.pow(a1+(a2*cos_theta2),2))-(math.pow(a2*sin_theta2,2)))
        i.theta1 = math.atan2(sin_theta1,cos_theta1)
        #finding theta4
        i.theta4 = theta123-i.theta1-i.theta2
        #finding b3
        i.b3 = z-b1-b2+b4

        joint_pub.publish(i)           #Publish joint's pose
        tool_pub.publish(p)            #Publish  tool frame's pose 

                        

if __name__ == '__main__':
    rospy.init_node('pose_listener', anonymous=True)
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
