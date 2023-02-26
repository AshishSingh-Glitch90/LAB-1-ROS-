#!/usr/bin/env python  
import roslib
roslib.load_manifest('lab1')
import rospy
import tf
from nav_msgs.msg import Odometry    
def handle_turtle_pose(msg, param):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z), (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w), rospy.Time.now(),param, "stage")
 
if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    param = rospy.get_param('~bot')
    rospy.Subscriber('/%s/odom' % param, Odometry, handle_turtle_pose, param)
    rospy.spin()