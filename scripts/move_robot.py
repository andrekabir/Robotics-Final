#!/usr/bin/env python3

import rospy
import copy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

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

class movement(object):
    def __init__(self):
        rospy.init_node("movement")
        self.map_topic = "map"
        self.pose_topic = "estimated_robot_pose"
        
        # For testing purposes
        # self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # inialize our map
        self.map = OccupancyGrid()
        self.pose = None
        # initialize robot velocity
        self.robot_speed = 0.2
        self.time = 2
        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        self.astar = AStarPlanner()
        rospy.sleep(4)
        self.path_poses = self.astar.get_path()

        rospy.Subscriber(self.pose_topic, PoseStamped, self.get_pose)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

#        self.delta_yaws = self.calculate_yaw_array()
        # define array times for robot to travel linearly & angularly (tuples)
        self.lin_ang_times = []
        self.index = 0
        rospy.sleep(1)

    def get_map(self, data):
        self.map = data
        

    def get_pose(self, data):
        self.pose = data.pose

########

# proportional control function -- how should we adjust angular velocity based on delta_yaw
#def set_ang_vel(self):
    #  radians/sec to turn 90 degrees in 4 seconds
    # 

# get estimated pose
# if each point in A* path is 'x' distance apart, then robot travels at (x/4) m/s
# self.map.info.resolution
# every 4 seconds, compare the robot's estimated pose with the point it should be at in the A* path array
# course correct by adjusting yaw so that robot's yaw matches that point's yaw and keep moving at (x/4) m/s.
    def course_correct_car(self):
        # find out delta yaw to course correct
        if self.pose == None:
            print("no pose")
            return
        print(self.index)
        print("length, ", len(self.path_poses))
        if self.index < len(self.path_poses):
            print("here")
            goal_yaw = self.path_poses[self.index]
            curr_yaw = self.pose
            delta_yaw = get_yaw_from_pose(goal_yaw)-get_yaw_from_pose(curr_yaw)
            delta_yaw = quaternion_from_euler(0.0, 0.0, delta_yaw)
            # publish an angular velocity so that robot rotates delta_yaw in 4 seconds (proportional control)
            new_ang_vel = delta_yaw/self.time
            move = Twist()
            move.linear.x = self.map.info.resolution/4
            move.angular.z = new_ang_vel

            self.cmd_vel_pub.publish(Twist)

            self.index += 1
        

        # publish lin velocity (always the same)
        # publish ang velocity (changes to course correct every time we rerun this code)
        # sleep for 4 seconds after publishing new ang velocity


########
#     def calc_contig_seg_travel_times(self):
#         # initial special case (to start off the x & y tot dists)
#         x_tot_dist = self.aster.poses[1].position.x-self.path_poses[0].postion.x
#         y_tot_dist = self.aster.poses[1].position.y-self.path_poses[0].postion.y
#         for index in range(1, len(self.delta_yaws)):
#             curr_yaw = self.delta_yaws[index]
#             prev_yaw = self.delta_yaws[index - 1]

#             if curr_yaw != prev_yaw:
#                 # append (lin, ang) distances to travel as a tuple of times 't' robot needs to move at its set speed
#                 # calculate hypotenuse
#                 hypotenuse = math.sqrt((x_tot_dist ** 2) + (y_tot_dist ** 2))
#                 lin_dist_time = hypotenuse/self.robot_speed

#                 ang_dist_time = 0 # SOMETHING ELSE HERE**
#                 self.lin_ang_times.append(lin_dist_time,ang_dist_time)
#                 x_tot_dist = 0
#                 y_tot_dist = 0
#                 #break
#             else:
#                 x_tot_dist += self.path_poses[index + 1].position.x - self.path_poses[index].position.x
#                 y_tot_dist += self.path_poses[index + 1].position.y - self.path_poses[index].position.y

            

# #   def make_time_array_for_lin_ang_directions

            
#             # rospy spin:
#             # publish lin velocity for duration time_array[x.0]
#             # publish ang velocity for duration time_array[x.1]

if __name__ == "__main__":
    movement_obj = movement()
    while not rospy.is_shutdown():
        movement_obj.course_correct_car()
        rospy.sleep(4)
