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

# Taken from paticle filter, to help work with quaternions
def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

# Compute the length of the hypotenuse between two points
def get_lin_dist_between_points(point1: Point, point2: Point):
    x1 = point1.x
    y1 = point1.y
    x2 = point2.x
    y2 = point2.y

    x_tot_dist = (x2 - x1)
    y_tot_dist = (y2 - y1)

    return math.sqrt((x_tot_dist ** 2) + (y_tot_dist ** 2))

# Find the difference in yaw between poses

def get_ang_dist_between_poses(pose1: Pose, pose2: Pose):
    yaw1 = get_yaw_from_pose(pose1)
    yaw2 = get_yaw_from_pose(pose2)
    return yaw2 - yaw1

# The main class for all of the movement

class movement(object):
    def __init__(self):
        rospy.init_node("movement")

        # Fix the speeds
        self.robot_linear_speed = 0.1
        self.robot_angular_speed = 0.1

        # Take in the AStarPlanner, wait for it to load, and then get the path
        self.astar = AStarPlanner()
        rospy.sleep(4)
        self.path_poses = self.astar.get_path()

        # Call later functions to get the linear distances and angular changes between points
        self.linear_distances, self.angular_distances = self.get_linear_distance_array(self.path_poses)

        # The current point that the robot is on
        self.curr_target_idx = 0

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Subscribe to the odom to figure out when we have traveled the desired distance
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.old_odom = None

        rospy.sleep(1)

    # Output the linear distances and angular changes between points, putting these in arrays
    def get_linear_distance_array(self, pose_array):
        lin_distance_arr = []
        ang_distance_arr = []
        num_poses = len(pose_array)

        # Go through all the poses to examine these changes between points
        for i in range(num_poses - 1):
            lin_dist = get_lin_dist_between_points(pose_array[i].position, pose_array[i+1].position)
            ang_dist = get_ang_dist_between_poses(pose_array[i], pose_array[i+1])
            lin_distance_arr.append(lin_dist)
            ang_distance_arr.append(ang_dist)
        
        return lin_distance_arr, ang_distance_arr

    # Taking in the data from the odom to determine our movement
    def odom_callback(self, data):
        # Current position
        curr_odom_position = data.pose.pose.position

        # If we're on the first point, assign the old_odom as the first point, i.e. the current odom position
        if self.old_odom is None:
            self.old_odom = curr_odom_position
        
        # Find the distance between the previous point, i.e. the old odom, and the current position
        distance_travelled = get_lin_dist_between_points(curr_odom_position, self.old_odom)

        # Check if we've travelled the required distance between these particular two points
        if distance_travelled > self.linear_distances[self.curr_target_idx]:
            # Make the robot stop
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(1)

            # Find the radians needed in the turn, sleep as long as necessary to complete the required turn
            radians = self.angular_distances[self.curr_target_idx]
            time_to_sleep = abs(radians / self.robot_angular_speed)

            # Make the robot rotate for the next movement, left or right depending on the radian sign
            twist_cmd = Twist()
            twist_cmd.angular.z = self.robot_angular_speed
            if radians < 0:
                twist_cmd.angular.z *= -1

            # If we have to rotate, publish the turn and sleep
            if radians != 0.0:
                self.cmd_vel_pub.publish(twist_cmd)
            rospy.sleep(time_to_sleep)

            # Update the pose we're looking how far we are from now, stop the robot from turning
            self.curr_target_idx += 1
            self.old_odom = curr_odom_position

            self.cmd_vel_pub.publish(Twist()) # stop the bot

            return
        
        # Make the robot move forward regardless of whether we needed to turn or not
        twist_cmd = Twist()
        twist_cmd.linear.x = self.robot_linear_speed
        self.cmd_vel_pub.publish(twist_cmd)

if __name__ == "__main__":
    movement_obj = movement()
    rospy.spin()
