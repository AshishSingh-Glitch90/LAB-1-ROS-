#! /usr/bin/python
#reference : https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
#reference : http://wiki.ros.org/Parameter%20Server
#reference : https://www.youtube.com/watch?v=-L-WgKMFuhE
import numpy as np
import rospy
import math
import message_filters
from tf import transformations
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
robot_loc = [0,0,0]
robot_ori = [0,0,0]
start_coor = None 
end_xy = None
path = []
stage = []
temp_index = 1
class Node():

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(map, start, end):
        
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    
    open_lst = []
    closed = []

    open_lst.append(start_node)
    
    while len(open_lst) > 0:        
        current = open_lst[0]
        current_index = 0
        for index, item in enumerate(open_lst):
            if item.f < current.f:
                current = item
                current_index = index

        
        open_lst.pop(current_index)
        closed.append(current)

        
        if current == end_node:
            path = []
            current = current
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] 

        
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: 

            
            node_position = (current.position[0] + new_position[0], current.position[1] + new_position[1])

            
            if node_position[0] > (len(map) - 1) or node_position[0] < 0 or node_position[1] > (len(map[len(map)-1]) -1) or node_position[1] < 0:
                continue

            
            if map[node_position[0]][node_position[1]] != 0:
                continue

            
            new_node = Node(current, node_position)

            
            children.append(new_node)

        
        for child in children:

            
            for closed_child in closed:
                if child == closed_child:
                    continue

            # calculating the f, g, and h values
            child.g = current.g + 1
            child.h = ((child.position[0] - end_node.position[0])**2) + ((child.position[1] - end_node.position[1])**2)*20
            child.f = child.g + child.h

            
            for open_node in open_lst:
                if child == open_node and child.g > open_node.g:
                    continue

            
            open_lst.append(child)
def distance(point1, point2):
    return math.sqrt((point2[1]- point1[1])**2 + (point2[0] - point1[0])**2)

pub= rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def robot_move(intial, ori, final):
    global temp_index, stage, path
    for x in path:
    angle = math.atan2((final[1]-x[1]),(final[0]-x[0]))
    dist = 
    err = angle-ori
    k = 3
    msg = Twist()
    if(dist > 0.08):
        msg.linear.x = 1
        if(abs(err)>0.01):
            msg.angular.z = k*err
            msg.linear.x = 0
        pub.publish(msg)
    else:
        print('----------------------'+str(stage[temp_index]))
        temp_index+=1
def callback(msg):
    global start_coor, end_xy, robot_loc, robot_ori
    odom_msg = msg.pose.pose
    robot_loc = (odom_msg.position.x, odom_msg.position.y)
    robot_ori_w = (odom_msg.orientation.x, odom_msg.orientation.y, odom_msg.orientation.z, odom_msg.orientation.w)
    (r,p,y) = transformations.euler_from_quaternion(robot_ori_w)
    robot_ori = [r,p,y]

def main():
    rospy.init_node('robot_movement')
    # rospy.Subscriber(name = '/odom', data_class = Odometry, callback = callback, queue_size= 1)
    global path, stage, start_coor, end_xy, robot_loc, robot_ori

    old_map  = np.array([0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
       0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1])

    map = old_map.reshape(20,18)
    start_coor = (-8.0,-2.0)
    end_coor_x = (rospy.get_param('goalx'))
    end_coor_y = (rospy.get_param('goaly'))
    end_xy = (end_coor_x, end_coor_y)

    #Matching the grid with the world map coordinates
    start = (int(round(start_coor[1]) + 14), int(round(start_coor[0])) + 9)
    end = (9 - int(round(end_coor_y)), int(math.floor(end_coor_x)) + 9)

    path = astar(map, start, end)
    for x in path:
        if x[1] == -4:
            x[1] = -5
        stage.append((x[1]-9, 10-x[0]))
    print(stage)
    for i in path:
        map[i[0]][i[1]] = 7
    print(map)
    print(path)
    

    while not rospy.is_shutdown():
        final = stage[temp_index]
        robot_move(robot_loc,robot_ori[2],final)
        

if __name__ == '__main__':
    main()
