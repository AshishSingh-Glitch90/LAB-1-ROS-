#! /usr/bin/python
import rospy
import math
import message_filters
from tf import transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

ini_pos = (-8.0, -2.0)
goalpos = (4.5, 9.0)

bot_laser= []
bot_pos = ()
bot_ori = ()
goal_seek= True

def callback(laser_msg, odom_msg):
    global bot_laser, bot_pos, bot_ori

    bot_laser= laser_msg.ranges
    bot_pos= (odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z)
    bot_ori= (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)

    bug2()

def bug2():
    robot_euler= euler_from_quaternion(bot_ori)
    m =  (goalpos[1] - ini_pos[1]) / (goalpos[0] - ini_pos[0])
    goal_angle = math.atan(m)
    tst_msg = Twist()
    final_angle = abs(robot_euler[2] - goal_angle)

    if (-0.1 < (robot_euler[2]-goal_angle) and (robot_euler[2]-goal_angle) < 0.1):
        if bot_laser[180] < 1.0:
            goal_seek = False
        else:
            
            tst_msg.linear.x = 1.0
            pub.publish(tst_msg)
    else:
        tst_msg.angular.z = final_angle
        pub.publish(tst_msg)

if __name__ =="__main__":
    print('Program starting')   
    rospy.init_node('bug2')
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    sub_1= message_filters.Subscriber('/base_scan', LaserScan)
    sub_2= message_filters.Subscriber('/odom', Odometry)
    sync= message_filters.TimeSynchronizer([sub_1, sub_2], 10)
    sync.registerCallback(callback)

    rospy.spin()
