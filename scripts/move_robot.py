#!/usr/bin/env python3

import rospy
import copy
import actionlib
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped, Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from nav_msgs.msg import Odometry
import math
import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import numpy as np
from numpy.random import random_sample
import math
from statistics import pstdev

from random import randint, random, uniform, choices, gauss

from path_planner import AStarPlanner

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def get_distance_between_points(point1: Point, point2: Point):
    x1 = point1.x
    y1 = point1.y
    x2 = point2.x
    y2 = point2.y

    x_tot_dist = (x2 - x1)
    y_tot_dist = (y2 - y1)

    return math.sqrt((x_tot_dist ** 2) + (y_tot_dist ** 2))
    


class movement(object):
    def __init__(self):
        rospy.init_node("movement")

        self.robot_speed = 0.1

        self.astar = AStarPlanner()
        rospy.sleep(4)
        self.path_poses = self.astar.get_path()
        print("PATH is as follows: ")

        self.linear_distances = self.get_distance_array(self.path_poses)
        self.curr_target_idx = 0
        # TODO Get angular distances similarly

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.old_odom = None

        rospy.sleep(1)

    def get_distance_array(self, pose_array):
        distance_arr = []
        num_poses = len(pose_array)
        for i in range(num_poses - 1):
            hypotenuse = get_distance_between_points(pose_array[i].position, pose_array[i+1].position)
            distance_arr.append(hypotenuse)
        return distance_arr


    def odom_callback(self, data):
        curr_odom_position = data.pose.pose.position
        if self.old_odom is None:
            self.old_odom = curr_odom_position
        
        distance_travelled = get_distance_between_points(curr_odom_position, self.old_odom)

        if distance_travelled > self.linear_distances[self.curr_target_idx]:
            self.curr_target_idx += 1
            self.old_odom = curr_odom_position
            print("Moved the first distance idx #", self.curr_target_idx)
            self.cmd_vel_pub.publish(Twist()) # stop the bot
            # TODO We need to turn here 
            rospy.sleep(4)

            return
        
        twist_cmd = Twist()
        twist_cmd.linear.x = self.robot_speed
        self.cmd_vel_pub.publish(twist_cmd)

"""
    def odom_callback_old(self, data):
        # print("ODOM Data", data)
        if not self.odom_flag:
            self.old_odom = data
            self.odom_flag = True
        new_odom = data
        
        # distance_moved = abs(self.old_odom.pose.pose.position.x - new_odom.pose.pose.position.x)
        distance_moved = get_distance_between_points(self.old_odom.pose.pose.position, new_odom.pose.pose.position)
        distance_to_move = self.distance_move(self.pose_idx)

        yaw_moved = abs(get_yaw_from_pose(self.old_odom.pose.pose) - get_yaw_from_pose(new_odom.pose.pose))
        yaw_to_move = self.yaw_move(self.pose_idx + 1)
        print()
        print("distance to move: ",distance_to_move,"distance_moved alr:",distance_moved)
        print("Yaw to move: ",yaw_to_move, "yaw moved alr:",yaw_moved)
        print("Following Point", self.pose_idx)
        # print("Poses Arr:", self.path_poses)

        
        
        if self.move_flag:
            print("Moving Forward")
            if (distance_moved > distance_to_move):
                my_twist = Twist(
                linear=Vector3(0.00, 0, 0),
                angular=Vector3(0, 0, 0.0)
                )
                self.cmd_vel_pub.publish(my_twist)
                # rospy.sleep(1)
              
                self.move_flag = False
            else:
                my_twist = Twist(
                linear=Vector3(0.05, 0, 0),
                angular=Vector3(0, 0, 0.0)
                )
                self.cmd_vel_pub.publish(my_twist)
        else:
            print("Turning")
            if (yaw_moved > yaw_to_move):
                my_twist = Twist(
                linear=Vector3(0.00, 0, 0),
                angular=Vector3(0, 0, 0.0)
                )
                self.cmd_vel_pub.publish(my_twist)
                rospy.sleep(5)
                self.pose_idx += 1
                self.move_flag = True
                self.old_odom = new_odom
                self.odom_flag = False

            else:
                if yaw_to_move > 0:
                    multi = -1
                else:
                    multi = 1
                my_twist = Twist(
                linear=Vector3(00, 0, 0),
                angular=Vector3(0, 0, 0.1*multi)
                )
                self.cmd_vel_pub.publish(my_twist)
                # rospy.sleep(1)
"""

        # print("distance_moved",distance_moved)


if __name__ == "__main__":
    movement_obj = movement()
    rospy.spin()
