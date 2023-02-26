#! /usr/bin/python
import math
import rospy
import tf
import random
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


rospy.init_node('perception')

pub = rospy.Publisher('/perception', Marker , queue_size = 1)
		

def cartesian(r , angle):
	y = r*math.sin(angle)
	x = r*math.cos(angle)
	return(x,y)

def ransac (points):
	threshold_pts = 30
	threshold = 0.25
	ransac_line = []

	while (len(points) > threshold_pts):
		max_inlier = 0
		best_inlier = []
		best_point = []
		for k in range(100):
			inlier_list = []
			inlier = 0
			point = random.sample(points, 2)
			m = (point[1][1] - point[0][1]) / (point[1][0] - point[0][0])
			c = point[1][1] - (m * point[1][0])
			i = 0
			for pt in points:
				dist = abs(m*pt[0] - pt[1] + c) / math.sqrt(m**2 + 1)
				
				if dist < threshold:
					inlier = inlier + 1
					inlier_list.append(pt)

				i = i + 1
		
			inlier = inlier - 2

			if inlier > max_inlier:
				best_inlier = inlier_list
				max_inlier = inlier
				best_point = point
		for total_pts in best_inlier:
			points.remove(total_pts)
		ransac_line.append(best_point)
			

	return (ransac_line)

def callback(msg):
	dist = msg.ranges
	angle = msg.angle_min
	increment = msg.angle_increment
	x = 0
	cartesian_list = []

	for r in dist:
		if r<3:
			cartesian_list.append(cartesian(r , angle + x * increment))
		x = x + 1
	
	best_point = ransac(cartesian_list)	
	
	message = Marker()
	message.header.stamp = rospy.Time.now()
	message.header.frame_id = "base_link"
	message.type = message.LINE_LIST
	message.action = message.ADD
	message.lifetime = rospy.Duration(10)
	message.scale.x = .1
	message.scale.y = .1
	message.color.a = 1.0
	message.color.r = 1.0
	for best in best_point:

		message.points.append(Point(best[0][0],best[0][1],0))
		message.points.append(Point(best[1][0],best[1][1],0))

	
	pub.publish(message)		

sub = rospy.Subscriber('/base_scan', LaserScan, callback)
rospy.spin()
