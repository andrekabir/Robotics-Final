#!/usr/bin/env python3

import rospy

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
#import matplotlib.pyplot as plt

import random
from likelihood_field import LikelihoodField
from typing import List


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""
    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def softmax(weights: List[float]) -> np.ndarray:
    """
    Normalizes the given weights using a softmax operation.
    """
    weights_array = np.array(weights)
    exp_values = np.exp(weights_array - np.max(weights_array))  # Shift by maximum value for better stability
    return exp_values / np.sum(exp_values)


def compute_prob_zero_centered_gaussian(dist, sd):
    """ 
    Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation. From class meeting 06.
    """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * np.exp(-1 * dist * dist) / (2 * sd * sd)
    return prob


class Particle:

    def __init__(self, pose: Pose, w: float):
        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w

    @property
    def angle(self) -> float:
        return get_yaw_from_pose(self.pose)

    def divide_weight(self, factor: float):
        assert factor != 0.0, 'Must provide nonzero factor'
        self.w = self.w / factor

    @classmethod
    def make(cls, x: float, y: float, angle: float, weight: float) -> Point:
        """
        Convenience method to make a point on a 2D plane with a single orientation angle (yaw / z)
        """
        point = Point(x=x, y=y, z=0.0)
        orientation = quaternion_from_euler(0.0, 0.0, angle)
        pose = Pose(position=point,
                    orientation=Quaternion(*orientation))
        return cls(pose=pose, w=weight)


class ParticleFilter:

    def __init__(self):
        # once everything is setup initialized will be set to true
        self.initialized = False        

        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 1000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        rospy.sleep(1)

    def get_map(self, data):
        self.map = data

        # Create the likelihood field using the given map
        self.likelihood_field = LikelihoodField(maze_map=self.map)

        # Initialize particle cloud after we get the map
        self.initialize_particle_cloud()

    def initialize_particle_cloud(self):
        if self.initialized:
            return

        def generate_angle(angle_multiple: int) -> float:
            return random.randint(0, 2 * angle_multiple) * math.pi / float(angle_multiple)
        
        # (1) Select random spots that are valid (e.g., not inside a wall)
        # (2) Can pick a random angle in [0, 2pi)
        # (3) Convert the angle + location to a proper pose
        # (4) Add to the point cloud

        valid_idxs = [idx for (idx, val) in enumerate(self.map.data) if val == 0]
        self.particle_cloud = []

        angle_multiple = 15.0  # Move in multiples if pi / 15

        if len(valid_idxs) < self.num_particles:
            # (1) Put a particle in every valid cell
            for p_idx in valid_idxs:
                x = self.map.info.resolution * int(p_idx % self.map.info.width) + self.map.info.origin.position.x
                y = self.map.info.resolution * int(p_idx / self.map.info.width) + self.map.info.origin.position.y
                theta = generate_angle(angle_multiple)

                assert (theta >= 0.0) and (theta <= 2 * math.pi), 'Invalid angle {}'.format(theta)

                particle = Particle.make(x=x, y=y, angle=theta, weight=1.0)
                self.particle_cloud.append(particle)

        # (2) For remaining, pick a random cell and put particle there
        num_remaining_particles = self.num_particles - len(self.particle_cloud)
        remaining_particle_idxs = random.choices(valid_idxs, k=num_remaining_particles)

        for p_idx in remaining_particle_idxs:
            x = self.map.info.resolution * int(p_idx % self.map.info.width) + self.map.info.origin.position.x
            y = self.map.info.resolution * int(p_idx / self.map.info.width) + self.map.info.origin.position.y
            theta = generate_angle(angle_multiple)

            assert (theta >= 0.0) and (theta <= 2 * math.pi), 'Invalid angle {}'.format(theta)

            particle = Particle.make(x=x, y=y, angle=theta, weight=1.0)
            self.particle_cloud.append(particle)

        # Normalize the weights and publish the particles + robot pose
        self.normalize_particles()
        self.publish_particle_cloud()
        self.update_estimated_robot_pose()
        self.publish_estimated_robot_pose()

        self.initialized = True

    def normalize_particles(self):
        # Compute the normalized weights
        weights = [particle.w for particle in self.particle_cloud]
        normalized_weights = softmax(weights)

        # Update the weights to their normalized values
        for idx, weight in enumerate(normalized_weights):
            self.particle_cloud[idx].w = weight

    def publish_particle_cloud(self):
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses = []

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)

    def publish_estimated_robot_pose(self):
        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)

    def resample_particles(self):
        # Get the particle weights
        weights = list(map(lambda particle: particle.w, self.particle_cloud))

        # Draw the random sample
        population_indices = list(range(self.num_particles))
        new_population_indices = np.random.choice(population_indices, size=self.num_particles, p=weights, replace=True)

        # Create the new particle cloud
        new_particle_cloud = [self.particle_cloud[idx] for idx in new_population_indices]
        self.particle_cloud = new_particle_cloud

    def robot_scan_received(self, data: LaserScan):
        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return

        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            dx = np.abs(curr_x - old_x)
            dy = np.abs(curr_y - old_y)
            dt = np.abs(curr_yaw - old_yaw)

            #print('Dx: {}, Dy: {}, DTheta: {}'.format(dx, dy, dt))

            if (dx > self.lin_mvmt_threshold or
                dy > self.lin_mvmt_threshold or
                dt > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out
                self.update_particles_with_motion_model()
                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()
                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose

    def is_out_of_bounds(self, particle: Particle) -> bool:
        """
        Using the given map, this function determines whether the particle is out of
        bounds with respect to the maze. A result of `True` means the particle is either outside
        of the maze or inside a wall. `False` indicates a valid position.
        """
        # Get the xy indices based on the map
        particle_x = particle.pose.position.x
        particle_y = particle.pose.position.y

        x_coord = int((particle_x - self.map.info.origin.position.x) / self.map.info.resolution)
        y_coord = int((particle_y - self.map.info.origin.position.y) / self.map.info.resolution)

        # Get the location's occupancy
        map_idx = int(x_coord + y_coord * self.map.info.width)
        occupancy = self.map.data[map_idx]

        # An occupancy of 0 indicates a valid, unoccupied location
        return occupancy != 0

    def update_estimated_robot_pose(self):
        # Keep a running sum of the x, y (position), and angle (orientation)
        x_sum, y_sum = 0.0, 0.0
        angle_sum = 0.0

        for particle in self.particle_cloud:
            x_sum += particle.pose.position.x
            y_sum += particle.pose.position.y
            angle_sum += particle.angle

        # Compute the average values for each field
        x_avg = x_sum / self.num_particles
        y_avg = y_sum / self.num_particles
        angle_avg = angle_sum / self.num_particles

        # Create the estimated pose
        orientation = quaternion_from_euler(0.0, 0.0, angle_avg)
        self.robot_estimate = Pose(position=Point(x=x_avg, y=y_avg, z=0.0),
                                  orientation=Quaternion(*orientation))

    def update_particle_weights_with_measurement_model(self, data: LaserScan):
        # Unpack the angles. We downsample for efficiency.
        angles = np.arange(0, len(data.ranges), 20, dtype=int)  # [D]
        measured_angles = np.array([math.radians(angle) for angle in angles])  # [D]
        #measured_distances = np.array([data.ranges[idx] for idx in angles])  # [D]

        # Clip the distances in to the valid range
        measured_distances = np.zeros(shape=(len(angles), ))

        for idx, angle_idx in enumerate(angles):
            dist = data.ranges[angle_idx]  # Get the measured distances at this angle

            if np.isnan(dist):
                dist = data.range_max  # Treat NaN distances as infinite and clip to the largest distances
            elif abs(dist) < 1e-7:
                dist = data.range_max  # On the turtlebot, a distance of 0 is out of range, so we use the largest distance

            dist = max(min(dist, data.range_max), data.range_min)  # Clip distances into the valid range
            measured_distances[idx] = dist

        # Process each particle
        for particle_idx, particle in enumerate(self.particle_cloud):
            if self.is_out_of_bounds(particle):
                particle.w = -1e7  # Use a large negative (log) weight if the particle is out of bounds
            else:
                # Unpack the current particle
                particle_x = particle.pose.position.x
                particle_y = particle.pose.position.y
                particle_angle = particle.angle

                # Rotate the position for each determined angle
                offset_angles = particle_angle + measured_angles
                x_rotated = particle_x + measured_distances * np.cos(offset_angles)  # [D]
                y_rotated = particle_y + measured_distances * np.sin(offset_angles)  # [D]

                # Use the likelihood field to get the closest obstacle distances for each rotated position
                distances = np.zeros(shape=(len(angles, )))  # [D]
                for idx in list(range(len(angles))):
                    dist = self.likelihood_field.get_closest_obstacle_distance(x=x_rotated[idx], y=y_rotated[idx])
                    distances[idx] = dist

                # Compute the update (log) weight. We use the log weight instead of the normal weight
                # for better numerical stability due to the sum (instead of product) aggregation.
                weights = compute_prob_zero_centered_gaussian(distances, sd=0.1)
                particle.w = np.sum(np.log(weights))
    
    def update_particles_with_motion_model(self):
        # Unpack the current and previous positions
        curr_x = self.odom_pose.pose.position.x
        curr_y = self.odom_pose.pose.position.y

        old_x = self.odom_pose_last_motion_update.pose.position.x
        old_y = self.odom_pose_last_motion_update.pose.position.y

        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        # Get the differences in position and angle
        dx, dy = curr_x - old_x, curr_y - old_y
        d_theta = curr_yaw - old_yaw
        displacement = math.sqrt(dx * dx + dy * dy)

        # Get the initial rotation angle for the Rotate-Translate-Rotate model
        init_rot_theta = np.arctan2(dy, dx) - old_yaw

        # Generate noise terms in batches for efficiency
        x_noise = np.random.uniform(low=-0.01, high=0.01, size=(self.num_particles, ))
        y_noise = np.random.uniform(low=-0.01, high=0.01, size=(self.num_particles, ))
        theta_noise = np.random.uniform(low=-0.01, high=0.01, size=(self.num_particles, ))

        updated_particles = []

        # Using rotate then translate for now.
        for particle_idx, particle in enumerate(self.particle_cloud):
            px, py = particle.pose.position.x, particle.pose.position.y
            p_theta = particle.angle

            # (1) First, rotate the particle. The particle will travel along this trajectory
            rot_theta = p_theta + init_rot_theta

            # (2) Then, translate the particle
            new_x = px + displacement * math.cos(rot_theta) + x_noise[particle_idx]
            new_y = py + displacement * math.sin(rot_theta) + y_noise[particle_idx]

            # (3) Finally, set the orientation after translation
            new_theta = p_theta + d_theta + theta_noise[particle_idx]

            new_particle = Particle.make(x=new_x, y=new_y, angle=new_theta, weight=particle.w)
            updated_particles.append(new_particle)

        self.particle_cloud = updated_particles


if __name__ == "__main__":
    pf = ParticleFilter()
    rospy.spin()
