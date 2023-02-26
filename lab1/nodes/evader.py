#! /usr/bin/python
import rospy
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
publish = Twist()
def callback(msg):
	dist = msg.ranges
	min_dist = min(dist)
	if (min_dist <= 2.0):
		publish.linear.x=0.0
		publish.angular.z=random.choice([20,45,60,75,90])
		# publish.angular.z = 1.0
		pub.publish(publish)
	else:
		publish.linear.x=2.0
		publish.angular.z=0.0
		pub.publish(publish)
	 
def subscriber():
	
	if param == 'evader':
		robot=rospy.Subscriber('/base_scan', LaserScan, callback)
	elif (param == "robot_0"):
		robot=rospy.Subscriber('/'+param+'/base_scan', LaserScan, callback)

	rospy.spin()
if __name__ =="__main__":
	global param
	rospy.init_node('evader')
	param = rospy.get_param('~bot')
	print (param)
	if param == 'evader':
		pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	elif (param == "robot_0") :
		pub = rospy.Publisher('/'+param+"/cmd_vel", Twist, queue_size=10)
	subscriber()

