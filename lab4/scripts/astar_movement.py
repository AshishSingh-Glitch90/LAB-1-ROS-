#! /usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time

def robot_move_to_goal (velocity_publisher):
    global a, b, yaw
    robot_velocity = Twist()
    path = [(-7, -2), (-6, -2), (-5, -2), (-4, -2), (-4, -3), (-3, -3), (-3, -4), (-2, -4), 
    (-1, -4), (0, -4), (1, -4), (2, -4), (3, -4), (3, -3), (3, -2), (3, -1), (3, 0), (4, 0), (4, 1), (4, 2), (4, 3), 
    (4, 4), (4, 5), (4, 6), (4, 7), (4, 8), (4, 9), (4, 10)]

    for s in path:
        while True:
            k_linear = 0.5
            distance = abs(math.sqrt(((s[0]-a)**2) + ((s[1] - b)**2)))
            linear_speed = distance * k_linear
            robot_velocity.linear.x = linear_speed
            angle_to_goal = math.atan2(s[1] - b, s[0] - a)
            k_angular = 4.0
            angular_speed = (angle_to_goal - yaw) * k_angular
            robot_velocity.angular.z = angular_speed
            velocity_publisher.publish(robot_velocity) #publishing both angular and linear velocity
            print('x =', a)
            print('y =', b)
            print('distance to goal =', distance)

            if (distance < 0.1):
                print("moving forward till reached")
                break

def posecallback(msg):
        global yaw, a, b
        odom_msg = msg.pose.pose
        a = odom_msg.position.x
        b = odom_msg.position.y
        yaw = odom_msg.orientation.w
        

if __name__ == '__main__':
    try:
        #print("hi i am here")
        rospy.init_node("robot_movementr", anonymous = True)
        robot_velocity = '/cmd_vel'
        velocity_publisher = rospy.Publisher(robot_velocity, Twist, queue_size=10)
        pose_subscriber = rospy.Subscriber(name = '/odom', data_class = Odometry, callback = posecallback)
        # robot_position = '/pose'
        # pose_subscriber = rospy.Subscriber(robot_position, Pose, posecallback)
        time.sleep(5)
        # move(velocity_publisher, 1 , 4 , False)
        # rotate(velocity_publisher, 30 , 90, False)
        robot_move_to_goal(velocity_publisher)
        # setdesiredorientation(velocity_publisher, 30, 90)
        # spiral(velocity_publisher, 0.5, 0)
        # gridclean(velocity_publisher)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")




