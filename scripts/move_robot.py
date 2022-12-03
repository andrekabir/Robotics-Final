#!/usr/bin/env python3

import rospy
import copy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
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

class movement(object):
    def __init__(self):
        rospy.init_node("movement")
        self.map_topic = "map"
        
        # For testing purposes
        # self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # inialize our map
        self.map = OccupancyGrid()
        # initialize robot velocity
        self.robot_speed = 0.2
        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        self.astar = AStarPlanner()
        rospy.sleep(3)

        self.delta_yaws = self.calculate_yaw_array()
        # define array times for robot to travel linearly & angularly (tuples)
        self.lin_ang_times = []
        rospy.sleep(1)

    def calculate_yaw_array(self):
        delta_yaws = []

        parray = self.astar.poses

        first_x = parray[0].position.x
        first_y = parray[0].position.y

        second_x = parray[1].position.x
        second_y = parray[1].position.y

        old_yaw = math.tan((second_y - first_y) / (second_x - first_x))

        for index in range(2, len(parray)):
            first_x = parray[index - 1].position.x
            first_y = parray[index - 1].position.y

            second_x = parray[index].position.x
            second_y = parray[index].position.y

            new_yaw = math.tan((second_y - first_y) / (second_x - first_x))

            delta_yaws.append(new_yaw - old_yaw)
            old_yaw = new_yaw

            # make array of yaw
            # iterate through yaws. as soon as prev_yaw != curr_yaw break
            # find next pose in pose arr for the corresponding yaw
            # calculate the hypotenuse (tot dist traveled) for the segment you just left
            # calculate duration (time) for robot to travel given its set speed and newly calculated hyp dist
            # robot should travel for 't' time at velocity 'v'.
            # after traveling for set duration, robot should rotate by delta yaw.
            # for the pose that corresponds with new yaw, now set x, y = 0 to start calculating a new hyp (the next contiguous segment)
            # keep going until you go through the whole pose array

    def calc_contig_seg_travel_times(self):
        # initial special case (to start off the x & y tot dists)
        x_tot_dist = self.aster.poses[1].position.x-self.astar.poses[0].postion.x
        y_tot_dist = self.aster.poses[1].position.y-self.astar.poses[0].postion.y
        for index in range(1, len(self.delta_yaws)):
            curr_yaw = self.delta_yaws[index]
            prev_yaw = self.delta_yaws[index - 1]

            if curr_yaw != prev_yaw:
                # append (lin, ang) distances to travel as a tuple of times 't' robot needs to move at its set speed
                # calculate hypotenuse
                hypotenuse = math.sqrt((x_tot_dist ** 2) + (y_tot_dist ** 2))
                lin_dist_time = hypotenuse/self.robot_speed
                ang_dist_time = 0 # SOMETHING ELSE HERE**
                self.lin_ang_times.append(lin_dist_time,ang_dist_time)
                x_tot_dist = 0
                y_tot_dist = 0
                #break
            else:
                x_tot_dist += self.astar.poses[index + 1].position.x - self.astar.poses[index].position.x
                y_tot_dist += self.astar.poses[index + 1].position.y - self.astar.poses[index].position.y

            

#   def make_time_array_for_lin_ang_directions

            
            # rospy spin:
            # publish lin velocity for duration time_array[x.0]
            # publish ang velocity for duration time_array[x.1]
