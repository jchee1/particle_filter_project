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

from random import randint, random
from likelihood_field import LikelihoodField

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample():
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """
    return

def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



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
        self.num_particles = 500

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

        #initialize likelihood field
        self.likelihood_field = LikelihoodField()

        # Let Map Subscribe
        rospy.sleep(5)

        # intialize the particle cloud
        self.initialize_particle_cloud()
        self.initialized = True



    def get_map(self, data):
        self.map = data
    

    def initialize_particle_cloud(self):
        width = self.map.info.width * self.map.info.resolution
        height = self.map.info.height * self.map.info.resolution
        x_origin = self.map.info.origin.position.x
        y_origin = self.map.info.origin.position.y
        x_bound, y_bound = self.likelihood_field.get_obstacle_bounding_box()
        #print(f"{width}, {height}")
        #print(f"{x_origin}, {y_origin}")
        #print(self.map.info.resolution)
        for i in range(self.num_particles):
            # Get Random Position
            #x = (width * random_sample()) + x_origin
            x = ((x_bound[1] - x_bound[0]) * random_sample()) + x_bound[0]
            y = ((y_bound[1] - y_bound[0])  * random_sample()) + y_bound[0]

            #y = (height * random_sample()) + y_origin
            # Get Random Orientation
            z_angular = (2 * np.pi) * random_sample()
            
            # Intialize Object
            p = Pose()
            p.position = Point()
            p.position.x = x
            p.position.y = y
            p.position.z = 0
            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, z_angular)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
            
            # initialize the new particle, where all will have the same weight (1.0)
            new_particle = Particle(p, 1.0)

            # append the particle to the particle cloud
            self.particle_cloud.append(new_particle)

        self.normalize_particles()

        self.publish_particle_cloud()
        #print(self.particle_cloud[0].pose.position.x, self.particle_cloud[0].pose.position.y)


    def normalize_particles(self):
        # make all the particle weights sum to 1.0
        
        #get total weights
        total_weights = 0
        for particle in self.particle_cloud:
            weight = particle.w
            total_weights += weight
        #print(f"Total Weights: {total_weights}")
        
        sm = 0
        #go through particles and normalize weights
        for i, particle in enumerate(self.particle_cloud):
            #print("working on particle:", particle)
            #print("Particle weight before: ", self.particle_cloud[i].w)
            self.particle_cloud[i].w /= total_weights
            #print("Particle weight after: ", self.particle_cloud[i].w)
            sm += self.particle_cloud[i].w
        
        #print("resample sum:", sm)


    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)


    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        weights = []
        #print("particle cloud before:", self.particle_cloud)
        for p in self.particle_cloud:
            weights.append(p.w)
        #print(f"weights: {weights} = {sum(weights)}")

        resample = np.random.choice(self.particle_cloud, len(self.particle_cloud), p=weights)
        for i, par in enumerate(resample):
            resample[i] = Particle(par.pose, par.w)
        self.particle_cloud = resample
        #print("particle cloud after:", self.particle_cloud)
        



    def robot_scan_received(self, data):

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


        if len(self.particle_cloud) > 0:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        # print("in update robot pose")
        # self.robot_estimate
        x_estimate = 0
        y_estimate = 0
        z_angular_estimate = 0

        sin_total = 0
        cos_total = 0
        
        #go through the particle cloud and take avgs of x, y, and z angular
        for particle in self.particle_cloud:
            x_estimate += particle.pose.position.x
            y_estimate += particle.pose.position.y
            yaw = get_yaw_from_pose(particle.pose)
            if yaw < 0:
                yaw += (np.pi * 2)
            sin_total += math.sin(yaw)
            cos_total += math.cos(yaw)
            # print(f"at {x_estimate}, {y_estimate} => yaw: {yaw}")
            

        x_estimate = x_estimate / len(self.particle_cloud)
        y_estimate = y_estimate / len(self.particle_cloud)
        z_angular_estimate = math.atan2(sin_total / len(self.particle_cloud),
            cos_total / len(self.particle_cloud))
        

        # print(f"Z-estimate: {z_angular_estimate}")

        pos = Point(x_estimate, y_estimate, 0)
        q = quaternion_from_euler(0.0, 0.0, z_angular_estimate)
        ori = Quaternion(q[0], q[1], q[2], q[3])
        
        
        self.robot_estimate = Pose(pos, ori)

    
    def update_particle_weights_with_measurement_model(self, data):
        
        # print("in measurement model")
        z = data.ranges
        #z = [0, 90, ]

        x_bound, y_bound = self.likelihood_field.get_obstacle_bounding_box()
        # print("x bound: ", x_bound)
        # print("y bound: ", y_bound)

        for i, particle in enumerate(self.particle_cloud):
            q = 1
            x = particle.pose.position.x
            y = particle.pose.position.y

            theta = get_yaw_from_pose(particle.pose)
            
            #print(particle)

            ztks = []
            dists = []
            probs = []

            angles = [0, 44, 89, 134, 179, 224, 269, 314]

            if x < x_bound[0] or x > x_bound[1] or y < y_bound[0] or y > y_bound[1]:
                # print(f"setting weight to 0")
                self.particle_cloud[i].w = 0
                continue

            for idx in angles:
                ztk = data.ranges[idx]
                # print("ztk:", ztk)
                #print("type is ", type(ztk))
                if (ztk <= data.range_max) and (ztk > 0): #may change to not equal instead
                    # print("in cond")
                    ztks.append(idx)
                    rad_idx = math.radians(idx)
                    x_z_tk = x + (ztk * math.cos(theta + rad_idx))
                    y_z_tk = y + (ztk * math.sin(theta + rad_idx))
                    dist = self.likelihood_field.get_closest_obstacle_distance(x_z_tk,y_z_tk)
                    dists.append(dist)
                    probs.append(compute_prob_zero_centered_gaussian(dist, 0.1))
                    q = q * compute_prob_zero_centered_gaussian(dist, 0.1)
            
            self.particle_cloud[i].w = q
        

    def update_particles_with_motion_model(self):

        # print("in motion model")
        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        delta = (curr_x - old_x, curr_y - old_y, curr_yaw - old_yaw)

        delta_rot1 = math.atan2(delta[1], delta[0]) - old_yaw
        delta_trans = math.sqrt(delta[0]**2 + delta[1]**2)
        delta_rot2 = delta[2] - delta_rot1

        for i, particle in enumerate(self.particle_cloud):
            hat_delta_rot1 = delta_rot1 - np.random.normal(0, 0.05) 
            hat_delta_trans = delta_trans - np.random.normal(0, 0.1) 
            hat_delta_rot2 = delta_rot2 - np.random.normal(0, 0.05) 

            particle_x = particle.pose.position.x
            particle_y = particle.pose.position.y
            particle_theta = get_yaw_from_pose(particle.pose)
            new_x = particle_x + hat_delta_trans * math.cos(particle_theta + hat_delta_rot1)
            new_y = particle_y + hat_delta_trans * math.sin(particle_theta + hat_delta_rot1)
            new_theta = particle_theta + hat_delta_rot1 + hat_delta_rot2

            p = Pose()
            p.position = Point()
            p.position.x = new_x
            p.position.y = new_y
            p.position.z = 0
            p.orientation = Quaternion()
            q = quaternion_from_euler(0.0, 0.0, new_theta)
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            #update in particle cloud
            self.particle_cloud[i].pose = p

            #print('particle_cloud:', self.particle_cloud)


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









