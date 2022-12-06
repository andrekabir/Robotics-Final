#!/usr/bin/env python3

import rospy
import copy
import sys
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

from likelihood_field import LikelihoodField

import heapq as hq

# Width of bot in metres
TURTLEBOT_WIDTH = 0.3

def make_pose_from_idx(valid_idx: int, info) -> Pose:
    resolution, width, origin = info.resolution, info.width, info.origin
    x = resolution * int(valid_idx % width) + origin.position.x
    y = resolution * int(valid_idx / width) + origin.position.y
    pose = make_pose(x=x, y=y, angle=0.0)
    return pose

     

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

"""
    Cell class: A unit on which A-star will operate
"""
class Cell(object):
    def __init__(self, x, y, obstacle_distance):
        self.pos_x, self.pos_y = x, y
        self.fx = math.inf
        self.gx = 0
        #print("Initing a cell with obstacle distance", obstacle_distance)
        if obstacle_distance < TURTLEBOT_WIDTH/2:
            self.hx = math.inf
        else:
            self.hx = 0
        self.explored = False
        self.parent = None
        self.open = False

    def get_pose(self, info) -> Pose:
        x = self.pos_x * info.resolution + info.origin.position.x
        y = self.pos_y * info.resolution + info.origin.position.y

        return make_pose(x=x, y=y, angle=0)

    def recalculate_fn(self):
        self.fx = self.gx + self.hx

    def __lt__(self, cell):
        if self.fx < cell.fx:
            return True
        elif self.fx == cell.fx:
            return self.hx < cell.hx
        else:
            return False


"""
    CellGraph
        - A graph data structure to perform A-Star on
"""
class CellGraph(object):

    """
        init: Takes in a map from the "map" ROS topic,
            Initializes 'Cell' objects in 2D array fashion for A-Star
    """
    def __init__(self, map):
        self.raw_map = map
        self.likelihood_field = LikelihoodField(maze_map=map)

        # Get the occupancy list
        occupancy_list = map.data
        #print("Occupancy list has size: ", len(occupancy_list))

        # Initialize 2D Array of cells corresponsing to valid occupancy values
        self.cell_array = {}
        self.experiment_poses = []

        valid_locs = len([i for i in occupancy_list if i >= 0])

        length = 0
        for (i, occupied) in enumerate(occupancy_list):
            if occupied != 0:
                continue # Location outside map

            x, y = int(i % map.info.width), int(i / map.info.width)
            #print("Valid pos: ", x, y)
            if x not in self.cell_array:
                self.cell_array[x] = {}

            self.cell_array[x][y] = Cell(x=x, y=y, obstacle_distance=self.likelihood_field.get_closest_obstacle_distance(x, y, transform=False))

            length += 1

        #print("Valid points: ", length)

        length = 0
        for l in self.cell_array:
            length += len(self.cell_array[l])

        #print("Number of cells in our graph: ", length)

        self.experiment_poses = [self.cell_array[180][115].get_pose(self.raw_map.info), self.cell_array[199][180].get_pose(self.raw_map.info)]

    """
        get_cell: Returns a cell given absolute coordinates (x, y)
    """
    def get_cell(self, x, y):
        if x not in self.cell_array or y not in self.cell_array[x]:
            return None

        return self.cell_array[x][y]

    def get_cell_from_pose(self, pose: Pose):
        x, y = self.make_coords_from_pose(pose)
        return self.get_cell(self, x, y)

    def make_coords_from_pose(self, pose: Pose):
        origin = self.raw_map.info.origin
        resolution = self.raw_map.resolution

        x = pose.position.x - origin.position.x
        y = pose.position.y - origin.position.y

        x //= resolution; y //= resolution

        return (x, y)

    def rms_distance(self, coord1, coord2):
        x_0, y_0 = coord1
        x_1, y_1 = coord2

        d = (x_0 - x_1) * (x_0 - x_1)
        d += (y_0 - y_1) * (y_0 - y_1)

        d = math.sqrt(d)
        return d

    def init_heuristic_cells(self, end_coord):
        # RMS distance
        for x in self.cell_array:
            for y in self.cell_array[x]:
                self.cell_array[x][y].hx += self.rms_distance((x, y), end_coord)
                self.cell_array[x][y].gx = 0
                self.cell_array[x][y].recalculate_fn()

    def get_path(self, start_coord, end_coord):
        print("AStarPlanner::get_path")
        start_cell = self.get_cell(start_coord[0], start_coord[1])
        end_cell = self.get_cell(end_coord[0], end_coord[1])

        if start_cell is None or end_cell is None:
            print("Terminals of path requested are invalid!")
            sys.exit(-1)

        self.init_heuristic_cells(end_coord)

        open_cells = []
        hq.heappush(open_cells, start_cell)

        current_cell = start_cell
        while True:
            current_cell = hq.heappop(open_cells) # should return cell with min fx
            if current_cell == end_cell:
                break

            #print("Current cell has fx: ", current_cell.fx)
            current_cell.explored = True
            cost_ngbr = 1

            for delta_x in [-1, 0, 1]:
                for delta_y in [-1, 0, 1]:
                    nx = current_cell.pos_x + delta_x
                    ny = current_cell.pos_y + delta_y

                    next_cell = self.get_cell(nx, ny)
                    if next_cell is None or next_cell.explored:
                        continue

                    new_gx = cost_ngbr + current_cell.gx
                    
                    if not next_cell.open:
                        next_cell.gx = new_gx
                        next_cell.recalculate_fn()
                        hq.heappush(open_cells, next_cell)
                        next_cell.open = True
                    else:
                        if next_cell.gx < new_gx:
                            continue
                        next_cell.gx = new_gx
                        next_cell.recalculate_fn()
                        hq.heapify(open_cells)
                    
                    next_cell.parent = current_cell
        
        tmp_cell = end_cell
        path = []
        while tmp_cell is not None:
            path.append(tmp_cell.get_pose(self.raw_map.info))
            tmp_cell = tmp_cell.parent
        
        # Change the path so that each pose points to the next pose in the path

        #first_x = path[0].position.x
        #first_y = path[0].position.y

        #second_x = path[1].position.x
        #second_y = path[1].position.y

        #if second_x == first_x:
        #    yaw = 0.0
        #else:
        #    yaw = math.tan((second_y - first_y) / (second_x - first_x))

        #converted_value = quaternion_from_euler(0.0, 0.0, yaw)
        #path[0].orientation = converted_value

        for index in range(1, len(path)):
            first_x = path[index - 1].position.x
            first_y = path[index - 1].position.y

            second_x = path[index].position.x
            second_y = path[index].position.y

            #print("first x: ", first_x)
            #print("first y: ", first_y)
            #print("second x: ", second_x)
            #print("second y: ", second_y)

            if second_x == first_x:
                yaw = -(math.pi/2)
            else:
                #print("here")
                if second_x < first_x:
                    yaw = math.pi + (math.atan((second_x - first_x) / (second_y - first_y)))
                else:
                    yaw = math.atan((second_x - first_x) / (second_y - first_y))

                #yaw = math.tan((second_y - first_y) / (second_x - first_x))
                #print(yaw)
            converted_value = quaternion_from_euler(0.0, 0.0, yaw)
            quat_value = Quaternion(converted_value[0], converted_value[1], converted_value[2], converted_value[3])
            path[index - 1].orientation = quat_value

        return path


"""
AStarPlanner - 
    A ROS Node that accepts a map (occupancy grid), start and end points 
    and genertates a shortest 'path' between the two points
"""
class AStarPlanner(object):
    def __init__(self):
        # rospy.init_node("path_planner")
        self.map_topic = "map"
        
        # For testing purposes
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        rospy.sleep(1)

        # inialize our map
        self.map = None

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        self.poses = []

        self.cell_graph = None

    """
    self.get_map() - callback to map_topic
        Takes in the map
        and generates a custom data structure, a 2D 'Cell' array 
        for performing A-star on this map
    """
    def get_map(self, data):
        print("AStarPlanner::get_map")

        self.map = data

    def get_path(self, start_coord=(185, 115), end_coord=(195, 180)):
        if self.map is None:
            print("Can not plan path, no map")
            sys.exit(-1)

        self.cell_graph = CellGraph(self.map)
        path = self.cell_graph.get_path(start_coord, end_coord)

        self.poses = path
        self.publish_poses()

        print("Done planning path, path pose len: ", len(self.poses))

        return self.poses


    def publish_poses(self):
        pose_array = PoseArray()
        pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        pose_array.poses = []

        for part in self.poses:
            pose_array.poses.append(part)

        self.particles_pub.publish(pose_array)
        #print("Number of valid points are: ", len(self.poses))


if __name__ == "__main__":
    pf = AStarPlanner()
    rospy.spin()
