#!/usr/bin/python
import roslib
roslib.load_manifest('lab1')
import rospy
import math
import tf
from geometry_msgs.msg import Twist
 
if __name__ == '__main__':
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
 
    turtle_vel = rospy.Publisher('robot_1/cmd_vel', Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/robot_1', '/robot_0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()