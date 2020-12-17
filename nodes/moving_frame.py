#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import tf2_ros
import tf_conversions
import geometry_msgs.msg

def publisher():
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    
    # periodic rate: 10 [hz]
    rate = rospy.Rate(10)
    # timer counter [second], increase 0.01 in each period
    tick = 0
    # angular velocity [rad/s]
    omega = 1.0
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        # This is target angle around z axis
        theta = omega*tick
        t.transform.translation.x = math.sin(theta)
        t.transform.translation.y = math.cos(theta)
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
        tick += 0.01
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('publisher', anonymous=True)

    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
