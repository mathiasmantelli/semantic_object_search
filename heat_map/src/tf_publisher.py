#!/usr/bin/env python  
import rospy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf
import geometry_msgs.msg
import tf_conversions

def callback(msg):
    br = TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w
    br.sendTransform(t)    


if __name__ == '__main__':
    rospy.init_node("my_python_tf_publisher", anonymous=True)
    rospy.Subscriber("/ground_truth_odom", Odometry, callback)
    rospy.spin()