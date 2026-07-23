#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class GroundTruthTFBroadcaster:
    def __init__(self):
        rospy.init_node('ground_truth_tf_broadcaster', anonymous=True)
        
        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribe to ground truth odometry
        self.odom_subscriber = rospy.Subscriber(
            '/ground_truth/state', 
            Odometry, 
            self.odom_callback
        )
        
        rospy.loginfo("Ground Truth TF Broadcaster started")
        
    def odom_callback(self, msg):
        """Convert odometry message to TF transform and broadcast it"""
        
        # Create transform message
        transform = TransformStamped()
        
        # Header
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id  # "world"
        transform.child_frame_id = msg.child_frame_id    # "base_link"
        
        # Translation
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        # Rotation
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        transform.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    try:
        broadcaster = GroundTruthTFBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass