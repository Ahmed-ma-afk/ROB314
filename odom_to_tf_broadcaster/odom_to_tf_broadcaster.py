#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry

def handle_odom_msg(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "/tello/base_link",  # This should be the child frame id from the odometry message
                     "/tello/local_origin") # This should be the frame id you're transforming from, typically "odom" or "map"

if __name__ == '__main__':
    rospy.init_node('odom_to_tf_broadcaster')
    rospy.Subscriber('/tello/odom', Odometry, handle_odom_msg)
    rospy.spin()
