#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert, Tomas

import os
import random
import math
import numpy
import time

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

from turtlebot3_msgs.srv import RingGoal
import xml.etree.ElementTree as ET
from ..drl_environment.drl_environment import ARENA_LENGTH, ARENA_WIDTH, ENABLE_DYNAMIC_GOALS
from ..common.settings import ENABLE_TRUE_RANDOM_GOALS

NO_GOAL_SPAWN_MARGIN = 0.3 # meters away from any wall
class DRLGazebo(Node):
    def __init__(self):
        super().__init__('drl_gazebo')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        # self.last_x = None
        # self.last_y = None
        # self.min_dist = 0.05  # or any threshold distance you want for path markers
  

        # self.entity_dir_path = (os.path.dirname(os.path.realpath(__file__))).replace(
        #     'turtlebot3_drl/lib/python3.8/site-packages/turtlebot3_drl/drl_gazebo',
        #     'turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_drl_world/goal_box')
        # self.entity_path = os.path.join(self.entity_dir_path, 'model.sdf')
        # self.entity = open(self.entity_path, 'r').read()
        # self.entity_name = 'goal'

        self.entity_dir_path = '/home/en-esmail/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_drl_world/goal_box'
        self.entity_path = os.path.join(self.entity_dir_path, 'model.sdf')
        self.entity = open(self.entity_path, 'r').read()
        self.entity_name = 'goal'

        # Path entity
        # self.path_entity_dir_path = '/home/en-esmail/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_drl_world/path_marker'
        # self.path_entity_path = os.path.join(self.path_entity_dir_path, 'model.sdf')
        # self.path_entity = open(self.path_entity_path, 'r').read()
        # self.path_entity_name = 'path'


        with open('/tmp/drlnav_current_stage.txt', 'r') as f:
            self.stage = int(f.read())
        print(f"running on stage: {self.stage}, dynamic goals enabled: {ENABLE_DYNAMIC_GOALS}")

        self.prev_x, self.prev_y = -1, -1
        self.goal_x, self.goal_y = 0.5, 0.0

        """************************************************************
        ** Initialise ROS publishers, subscribers and clients
        ************************************************************"""
        # Initialise publishers
        self.goal_pose_pub = self.create_publisher(Pose, 'goal_pose', QoSProfile(depth=10))

        # Initialise client
        self.delete_entity_client       = self.create_client(DeleteEntity, 'delete_entity')
        self.spawn_entity_client        = self.create_client(SpawnEntity, 'spawn_entity')
        self.reset_simulation_client    = self.create_client(Empty, 'reset_simulation')
        self.gazebo_pause               = self.create_client(Empty, '/pause_physics')

        # Initialise servers
        self.task_succeed_server    = self.create_service(RingGoal, 'task_succeed', self.task_succeed_callback)
        self.task_fail_server       = self.create_service(RingGoal, 'task_fail', self.task_fail_callback)

        self.obstacle_coordinates   = self.get_obstacle_coordinates()
        # self.init_callback()

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)



        # Goal marker publisher
        self.goal_pub = self.create_publisher(Marker, "goal_marker", 10)

        # Configure goal marker
        self.goal_marker = Marker()
        self.goal_marker.header.frame_id = "odom"   # or "map"
        self.goal_marker.ns = "goal"
        self.goal_marker.id = 0
        self.goal_marker.type = Marker.CYLINDER
        self.goal_marker.action = Marker.ADD
        self.goal_marker.scale.x = 0.3
        self.goal_marker.scale.y = 0.3
        self.goal_marker.scale.z = 0.05
        self.goal_marker.color.a = 1.0
        self.goal_marker.color.g = 1.0

        # Subscriber
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Marker, "/path_marker", 10)
        self.optimal_pub = self.create_publisher(Marker, "/optimal_path_marker", 10)

        # Store actual path (LINE_STRIP)
        self.path_marker = Marker()
        self.path_marker.header.frame_id = "odom"
        self.path_marker.type = Marker.LINE_STRIP
        self.path_marker.action = Marker.ADD
        self.path_marker.scale.x = 0.02
        self.path_marker.color.r = 1.0
        self.path_marker.color.g = 0.0
        self.path_marker.color.b = 0.0
        self.path_marker.color.a = 1.0

        # Store optimal path (predefined, example here)
        self.optimal_path_marker = Marker()
        self.optimal_path_marker.header.frame_id = "odom"
        self.optimal_path_marker.type = Marker.LINE_STRIP
        self.optimal_path_marker.action = Marker.ADD
        self.optimal_path_marker.scale.x = 0.02
        self.optimal_path_marker.color.r = 0.0
        self.optimal_path_marker.color.g = 0.0
        self.optimal_path_marker.color.b = 1.0
        self.optimal_path_marker.color.a = 1.0

        # Example static path (replace with your planner output)
        p1 = Point(); p1.x = 0.0; p1.y = 0.0
        p2 = Point(); p2.x = 1.0; p2.y = 0.0
        p3 = Point(); p3.x = 2.0; p3.y = 1.0
        self.optimal_path_marker.points = [p1, p2, p3]



        # # Path marker publisher
        # self.path_pub = self.create_publisher(Marker, "path_marker", 10)

        # # Configure path marker
        # self.path_marker = Marker()
        # self.path_marker.header.frame_id = "odom"
        # self.path_marker.type = Marker.LINE_STRIP
        # self.path_marker.action = Marker.ADD
        # self.path_marker.scale.x = 0.02             # line thickness
        # self.path_marker.color.a = 1.0              # alpha
        # self.path_marker.color.r = 1.0              # red
        # self.path_marker.points = []

        # # Publisher for planned path marker
        # self.optimal_path_pub = self.create_publisher(Marker, "optimal_path_marker", 10)

        # # Marker configuration for optimal path
        # self.optimal_path_marker = Marker()
        # self.optimal_path_marker.header.frame_id = "odom"   # Nav2 works in map frame
        # self.optimal_path_marker.type = Marker.LINE_STRIP
        # self.optimal_path_marker.action = Marker.ADD
        # self.optimal_path_marker.scale.x = 0.03
        # self.optimal_path_marker.color.a = 1.0
        # self.optimal_path_marker.color.b = 1.0   # Blue for planned path
        # self.optimal_path_marker.points = []

        # # Subscribe to planned path
        # self.create_subscription(Path, '/plan', self.planned_path_callback, 10)



        self.init_callback()



        # # In __init__ method of DRLGazebo
        # self.planned_path_pub = self.create_publisher(Path, '/planned_path', 10)
        # self.actual_path_pub  = self.create_publisher(Path, '/actual_path', 10)

        # self.create_subscription(Path, '/plan', self.planned_path_callback, 10)
        # self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # self.actual_path = Path()
        # self.actual_path.header.frame_id = "map"


        

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""

    def init_callback(self):
        self.delete_entity()
        self.reset_simulation()
        self.publish_callback()
        print("Init, goal pose:", self.goal_x, self.goal_y)
        time.sleep(1)

    def publish_callback(self):
        # Publish goal pose
        goal_pose = Pose()
        goal_pose.position.x = self.goal_x
        goal_pose.position.y = self.goal_y
        self.goal_pose_pub.publish(goal_pose)
        self.spawn_entity()

    # def spawn_path_marker(self, x, y):
    #     path_pose = Pose()
    #     path_pose.position.x = x
    #     path_pose.position.y = y
    #     req = SpawnEntity.Request()
    #     req.name = f"{self.path_entity_name}_{x}_{y}"  # unique name per marker
    #     req.xml = self.path_entity
    #     req.initial_pose = path_pose
    #     while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('service not available, waiting again...')
    #     self.spawn_entity_client.call_async(req)
    #     self.get_logger().info(f"Spawned path marker at ({x}, {y})")

    # def spawn_path_segment(self, x, y):
    #     sdf_path = "/home/en-esmail/ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_drl_world/path_marker/model.sdf"
    #     entity_xml = open(sdf_path, 'r').read()
    #     entity_name = f"path_{int(time.time()*1000)}"  # unique name per segment

    #     request = SpawnEntity.Request()
    #     request.name = entity_name
    #     request.xml = entity_xml
    #     request.robot_namespace = ""
    #     request.initial_pose.position.x = x
    #     request.initial_pose.position.y = y
    #     request.initial_pose.position.z = 0.02

    #     self.spawn_entity_client.call_async(request)



    # def odom_callback(self, msg):
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
        
    #     p = Point()
    #     p.x, p.y, p.z = x, y, 0.05   # small z lift so line doesn’t clip the floor
    #     self.path_marker.points.append(p)

    #     # Update header and publish
    #     self.path_marker.header.stamp = self.get_clock().now().to_msg()
    #     self.marker_pub.publish(self.path_marker)

    # def odom_callback(self, msg):
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y

    #     # Only spawn if robot moved more than min_dist
    #     if self.last_x is None or self.last_y is None:
    #         self.spawn_path_segment(x, y)
    #         self.last_x, self.last_y = x, y
    #         return

    #     dx = x - self.last_x
    #     dy = y - self.last_y
    #     dist = (dx**2 + dy**2)**0.5

    #     if dist > self.min_dist:
    #         self.spawn_path_segment(x, y)
    #         self.last_x, self.last_y = x, y


    # def planned_path_callback(self, msg: Path):
    #     # Clear previous path
    #     self.optimal_path_marker.points = []

    #     # Convert Path into Marker points
    #     for pose_stamped in msg.poses:
    #         p = Point()
    #         p.x = pose_stamped.pose.position.x
    #         p.y = pose_stamped.pose.position.y
    #         p.z = 0.05
    #         self.optimal_path_marker.points.append(p)

    #     # Publish marker
    #     self.optimal_path_marker.header.stamp = self.get_clock().now().to_msg()
    #     self.optimal_path_pub.publish(self.optimal_path_marker)


    # def odom_callback(self, msg):
    #     p = Point()
    #     p.x = msg.pose.pose.position.x
    #     p.y = msg.pose.pose.position.y
    #     p.z = 0.05
    #     self.path_marker.points.append(p)
    #     self.path_marker.header.stamp = self.get_clock().now().to_msg()
    #     self.path_pub.publish(self.path_marker)



    def odom_callback(self, msg: Odometry):
        # Update actual path with robot’s current position
        p = Point()
        p.x = msg.pose.pose.position.x
        p.y = msg.pose.pose.position.y
        p.z = 0.0
        self.path_marker.points.append(p)

        # Update headers
        self.path_marker.header.stamp = self.get_clock().now().to_msg()
        self.optimal_path_marker.header.stamp = self.get_clock().now().to_msg()

        # Publish both
        self.path_pub.publish(self.path_marker)
        self.optimal_pub.publish(self.optimal_path_marker)







    

        # __________________________________________________

    # def planned_path_callback(self, msg):
    #     # Republish planned path for visualization
    #     self.planned_path_pub.publish(msg)

    # def odom_callback(self, msg):
    #     # Build actual path from odometry
    #     pose_stamped = PoseStamped()
    #     pose_stamped.header = msg.header
    #     pose_stamped.pose = msg.pose.pose
    #     self.actual_path.poses.append(pose_stamped)

    #     # Limit path length
    #     if len(self.actual_path.poses) > 1000:
    #         self.actual_path.poses.pop(0)

    #     self.actual_path_pub.publish(self.actual_path)
    
    # __________________________________________________




    def task_succeed_callback(self, request, response):
        self.delete_entity()
        if ENABLE_TRUE_RANDOM_GOALS:
            self.generate_random_goal()
            print(f"success: generate (random) a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        elif ENABLE_DYNAMIC_GOALS:
            self.generate_dynamic_goal_pose(request.robot_pose_x, request.robot_pose_y, request.radius)
            print(f"success: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}, radius: {request.radius:.2f}")
        else:
            self.generate_goal_pose()
            print(f"success: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response

    def task_fail_callback(self, request, response):
        self.delete_entity()
        self.reset_simulation()
        if ENABLE_TRUE_RANDOM_GOALS:
            self.generate_random_goal()
            print(f"fail: reset the environment, (random) goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        elif ENABLE_DYNAMIC_GOALS:
            self.generate_dynamic_goal_pose(request.robot_pose_x, request.robot_pose_y, request.radius)
            print(f"fail: reset the environment, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}, radius: {request.radius:.2f}")
        else:
            self.generate_goal_pose()
            print(f"fail: reset the environment, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response

    def goal_is_valid(self, goal_x, goal_y):
        if goal_x > ARENA_LENGTH/2 or goal_x < -ARENA_LENGTH/2 or goal_y > ARENA_WIDTH/2 or goal_y < -ARENA_WIDTH/2:
            return False
        for obstacle in self.obstacle_coordinates:
            if goal_x < obstacle[0][0] and goal_x > obstacle[2][0]:
                if goal_y < obstacle[0][1] and goal_y > obstacle[2][1]:
                    return False
        return True

    def generate_random_goal(self):
        self.prev_x = self.goal_x
        self.prev_y = self.goal_y
        tries = 0
        while (((abs(self.prev_x - self.goal_x) + abs(self.prev_y - self.goal_y)) < 4) or (not self.goal_is_valid(self.goal_x, self.goal_y))):
            self.goal_x = random.randrange(-25, 25) / 10.0
            self.goal_y = random.randrange(-25, 25) / 10.0
            tries += 1
            if tries > 200:
                print("ERROR: cannot find valid new goal, resestting!")
                self.delete_entity()
                self.reset_simulation()
                self.generate_goal_pose()
                break
        self.publish_callback()

    def generate_dynamic_goal_pose(self, robot_pose_x, robot_pose_y, radius):
        tries = 0
        while(True):
            ring_position = random.uniform(0, 1)
            origin = radius + numpy.random.normal(0, 0.1) # in meters
            goal_offset_x = math.cos(2 * math.pi * ring_position) * origin
            goal_offset_y = math.sin(2 * math.pi * ring_position) * origin
            goal_x = robot_pose_x + goal_offset_x
            goal_y = robot_pose_y + goal_offset_y
            if self.goal_is_valid(goal_x, goal_y):
                self.goal_x = goal_x
                self.goal_y = goal_y
                break
            if tries > 100:
                print("Error! couldn't find valid goal position, resetting..")
                self.delete_entity()
                self.reset_simulation()
                self.generate_goal_pose()
                return
            tries += 1
        self.publish_callback()


    # def generate_goal_pose(self):
    #     self.prev_x = self.goal_x
    #     self.prev_y = self.goal_y
    #     tries = 0

    #     while ((abs(self.prev_x - self.goal_x) + abs(self.prev_y - self.goal_y)) < 2):
    #         if self.stage == 11:
    #             # --- Define static goal positions here ---
    #             goal_pose_list = [[0.0, 0.0], [0.0, 6.5], [5.0, 5.5], [-2.5, -6.0], [3.0, -4.0], [6.0, -1.0]]
    #             index = random.randrange(0, len(goal_pose_list))
    #             self.goal_x = float(goal_pose_list[index][0])
    #             self.goal_y = float(goal_pose_list[index][1])
    #         elif self.stage == 8 or self.stage == 9 or self.stage == 12:
    #             # --- Define static goal positions here ---
    #             goal_pose_list = [[2.0, 2.0], [2.0, 1.5], [2.0, -0.5], [2.0, -1.0], [2.0, -2.0], [1.3, 1.0],
    #                                 [1.0, 0.3], [1.0, -2.0], [0.3, -1.0],  [0.0, 2.0], [0.0, -1.0], [-1.0, 1.0],
    #                                     [-1.0, -1.2], [-2.0, 1.0], [-2.2, 0.0], [-2.0, -2.2], [-2.4, 2.4]]
    #             index = random.randrange(0, len(goal_pose_list))
    #             self.goal_x = float(goal_pose_list[index][0])
    #             self.goal_y = float(goal_pose_list[index][1])
    #         elif self.stage not in [4, 5, 7]:
    #             self.goal_x = random.randrange(-15, 16) / 10.0
    #             self.goal_y = random.randrange(-15, 16) / 10.0
    #         else:
    #             # --- Define static goal positions here ---
    #             goal_pose_list = [[1.0, 0.0], [2.0, -1.5], [0.0, -2.0], [2.0, 2.0], [0.8, 2.0],
    #                               [-1.9, 1.9], [-1.9,  0.2], [-1.9, -0.5], [-2.0, -2.0], [-0.5, -1.0],
    #                               [1.5, -1.0], [-0.5, 1.0], [-1.0, -2.0], [1.8, -0.2], [1.0, -1.9]]
    #             index = random.randrange(0, len(goal_pose_list))
    #             self.goal_x = float(goal_pose_list[index][0])
    #             self.goal_y = float(goal_pose_list[index][1])
    #         tries += 1
    #         if tries > 100:
    #             print("ERROR: distance between goals is small!")
    #             break
    #     self.publish_callback()

    def generate_goal_pose(self):
        # Save previous goal
        self.prev_x = self.goal_x
        self.prev_y = self.goal_y
        tries = 0

        # --- Force the very first goal to (2,1) only if stage == 8 ---
        if self.stage == 8 and not hasattr(self, "first_goal_set"):
            self.goal_x = 1.0
            self.goal_y = 1.0
            self.first_goal_set = True  # Mark that we've already set it
        else:
            while ((abs(self.prev_x - self.goal_x) + abs(self.prev_y - self.goal_y)) < 2):
                if self.stage == 11:
                    goal_pose_list = [[0.0, 0.0], [0.0, 6.5], [5.0, 5.5], [-2.5, -6.0], [3.0, -4.0], [6.0, -1.0]]
                    index = random.randrange(0, len(goal_pose_list))
                    self.goal_x = float(goal_pose_list[index][0])
                    self.goal_y = float(goal_pose_list[index][1])
                elif self.stage in [8, 9, 12]:
                    goal_pose_list = [[2.0, 2.0], [2.0, 1.5], [2.0, -0.5], [2.0, -1.0], [2.0, -2.0], [1.3, 1.0],
                                    [1.0, 0.3], [1.0, -2.0], [0.3, -1.0], [0.0, 2.0], [0.0, -1.0], [-1.0, 1.0],
                                    [-1.0, -1.2], [-2.0, 1.0], [-2.2, 0.0], [-2.0, -2.2], [-2.4, 2.4]]
                    index = random.randrange(0, len(goal_pose_list))
                    self.goal_x = float(goal_pose_list[index][0])
                    self.goal_y = float(goal_pose_list[index][1])
                elif self.stage not in [4, 5, 7]:
                    self.goal_x = random.randrange(-15, 16) / 10.0
                    self.goal_y = random.randrange(-15, 16) / 10.0
                else:
                    goal_pose_list = [[1.0, 0.0], [2.0, -1.5], [0.0, -2.0], [2.0, 2.0], [0.8, 2.0],
                                    [-1.9, 1.9], [-1.9, 0.2], [-1.9, -0.5], [-2.0, -2.0], [-0.5, -1.0],
                                    [1.5, -1.0], [-0.5, 1.0], [-1.0, -2.0], [1.8, -0.2], [1.0, -1.9]]
                    index = random.randrange(0, len(goal_pose_list))
                    self.goal_x = float(goal_pose_list[index][0])
                    self.goal_y = float(goal_pose_list[index][1])

                tries += 1
                if tries > 100:
                    print("ERROR: distance between goals is small!")
                    break

        self.publish_callback()


    def reset_simulation(self):
        req = Empty.Request()
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset service not available, waiting again...')
        self.reset_simulation_client.call_async(req)

    def delete_entity(self):
        req = DeleteEntity.Request()
        req.name = self.entity_name
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.delete_entity_client.call_async(req)

    def spawn_entity(self):
        goal_pose = Pose()
        goal_pose.position.x = self.goal_x
        goal_pose.position.y = self.goal_y
        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = self.entity
        req.initial_pose = goal_pose
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.spawn_entity_client.call_async(req)

        self.goal_marker.header.stamp = self.get_clock().now().to_msg()
        self.goal_marker.pose.position.x = self.goal_x
        self.goal_marker.pose.position.y = self.goal_y
        self.goal_marker.pose.position.z = 0.05

        self.goal_pub.publish(self.goal_marker)


        



    def get_obstacle_coordinates(self):
        tree = ET.parse(os.getenv('DRLNAV_BASE_PATH') + '/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/inner_walls/model.sdf')
        root = tree.getroot()
        obstacle_coordinates = []
        for wall in root.find('model').findall('link'):
            pose = wall.find('pose').text.split(" ")
            size = wall.find('collision').find('geometry').find('box').find('size').text.split()
            rotation = float(pose[-1])
            pose_x = float(pose[0])
            pose_y = float(pose[1])
            if rotation == 0:
                size_x = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
                size_y = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
            else:
                size_x = float(size[1]) + NO_GOAL_SPAWN_MARGIN * 2
                size_y = float(size[0]) + NO_GOAL_SPAWN_MARGIN * 2
            point_1 = [pose_x + size_x / 2, pose_y + size_y / 2]
            point_2 = [point_1[0], point_1[1] - size_y]
            point_3 = [point_1[0] - size_x, point_1[1] - size_y ]
            point_4 = [point_1[0] - size_x, point_1[1] ]
            wall_points = [point_1, point_2, point_3, point_4]
            obstacle_coordinates.append(wall_points)
        return obstacle_coordinates


def main():
    rclpy.init()
    drl_gazebo = DRLGazebo()
    rclpy.spin(drl_gazebo)

    drl_gazebo.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()