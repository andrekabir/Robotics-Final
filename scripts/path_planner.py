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

from random import randint, random, uniform, choices, gauss

# from Robotics_Final.msg import r1
# from Robotics_Final.msg import r2

from likelihood_field import *


def make_pose_from_idx(valid_idx: int, resolution, origin, width) -> Pose:
    x = resolution * int(valid_idx % width) + origin.position.x
    y = resolution * int(valid_idx / width) + origin.position.y
    pose = make_pose(x=x, y=y, angle=0.0)
    return pose

def make_idx_from_pose(pose,resolution, origin, width):
    # x = pose.position.x
    y = pose.position.y

    idx = int((y - origin.position.y)/resolution) * width 
    return idx 

     

def make_pose(x: float, y: float, angle: float) -> Pose:
    """
    Convenience method to make a point on a 2D plane with a single orientation angle (yaw / z)
    """
    point = Point(x=x, y=y, z=0.0)
    orientation = quaternion_from_euler(0.0, 0.0, angle)
    pose = Pose(position=point,
                orientation=Quaternion(*orientation))

    return pose
    
def explore_neighbours(pose):
    
    neighbours = []

    for xd,yd in [(0,0.1),(-0.1,0),(0,-0.1),(.1,0)]:
        neighbours.append(make_pose(pose.position.x+xd, 
                                    pose.position.y+yd,
                                    0))
    return neighbours

class Cell(object):
    def __init__(self,idx):
        self.fx = 100
        self.gx = 100
        self.hx = 100
        self.index = idx
        self.explored = False
    
    # def explore_neighbours(self):
        
    #     neighbours = []

    #     for xd,yd in [(0,0.1),(-0.1,0),(0,-0.1),(.1,0)]:
    #         neighbours.append(make_pose(   self.pose.position.x+xd, 
    #                                         self.pose.position.y+yd,
    #                                         0))
    #     return neighbours




class AStarPlanner(object):
    def __init__(self):
        rospy.init_node("path_planner")
        self.map_topic = "map"
        
        # For testing purposes
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        rospy.sleep(1)

        # inialize our map
        self.map = OccupancyGrid()

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        self.poses = []

    def get_map(self, data):
        self.map = data

        print("\n\nFetched map occupancy data, size: ", len(self.map.data))
        # print(self.map.data)

        valid_idxs = [idx for (idx, val) in enumerate(self.map.data) if val == 0]

        cell_list = []

        for n in valid_idxs:
            cell_list.append(Cell(n))
        print(cell_list)

        
        self.poses = []

        # for p_idx in valid_idxs:
        #     x = self.map.info.resolution * int(p_idx % self.map.info.width) + self.map.info.origin.position.x
        #     y = self.map.info.resolution * int(p_idx / self.map.info.width) + self.map.info.origin.position.y
        #     # theta = generate_angle(angle_multiple)

        #     # assert (theta >= 0.0) and (theta <= 2 * math.pi), 'Invalid angle {}'.format(theta)

        #     pose = make_pose(x=x, y=y, angle=0.0)
        #     self.poses.append(pose)

        # self.publish_poses()
        start_idx, end_idx = valid_idxs[1000], valid_idxs[-10]
        first_pose = make_pose_from_idx(start_idx, self.map.info.resolution, self.map.info.origin, self.map.info.width)
        print("Index",make_idx_from_pose(first_pose,self.map.info.resolution, self.map.info.origin, self.map.info.width),  start_idx)
        neighbours = explore_neighbours(first_pose)
        self.poses = neighbours
        self.poses.append(first_pose)
        # [make_pose_from_idx(start_idx, self.map.info.resolution, self.map.info.origin, self.map.info.width), 
                    # make_pose_from_idx(end_idx, self.map.info.resolution, self.map.info.origin, self.map.info.width)]
        
        
        self.publish_poses()

    def publish_poses(self):
        pose_array = PoseArray()
        pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        pose_array.poses = []

        for part in self.poses:
            pose_array.poses.append(part)

        self.particles_pub.publish(pose_array)
        print("Number of valid points are: ", len(self.poses))


if __name__ == "__main__":
    pf = AStarPlanner()
    rospy.spin()
