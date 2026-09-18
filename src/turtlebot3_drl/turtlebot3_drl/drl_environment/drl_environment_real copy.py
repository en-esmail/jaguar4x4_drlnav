#
#!/usr/bin/env python3
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

import math
import numpy
import sys
import copy
import numpy as np



import collections

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from turtlebot3_msgs.srv import DrlStep, Goal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data


# Bug: reward function
from . import reward as rw

from ..common import utilities as util
from ..common.settings import ENABLE_BACKWARD, UNKNOWN, SUCCESS, COLLISION_WALL, REAL_ARENA_LENGTH, REAL_ARENA_WIDTH, \
                                REAL_SPEED_LINEAR_MAX, REAL_SPEED_ANGULAR_MAX, REAL_N_SCAN_SAMPLES, REAL_LIDAR_DISTANCE_CAP, REAL_LIDAR_CORRECTION, \
                                    REAL_THRESHOLD_COLLISION, REAL_THRESHOLD_GOAL, REAL_TOPIC_SCAN, REAL_TOPIC_VELO, REAL_TOPIC_ODOM

LINEAR = 0
ANGULAR = 1

MAX_GOAL_DISTANCE = math.sqrt(REAL_ARENA_LENGTH**2 + REAL_ARENA_WIDTH**2)

class DRLEnvironment(Node):
    def __init__(self):
        super().__init__('drl_environment')
        print(f"running on real stage")

        self.scan_topic = REAL_TOPIC_SCAN
        self.velo_topic = REAL_TOPIC_VELO
        self.odom_topic = REAL_TOPIC_ODOM
        self.goal_topic = 'goal_pose'

        self.goal_x, self.goal_y = 0.0, 0.0
        self.robot_x, self.robot_y = 0.0, 0.0
        self.robot_x_prev, self.robot_y_prev = 0.0, 0.0
        self.robot_heading = 0.0
        self.total_distance = 0.0
        self.robot_tilt = 0.0

        self.done = False
        self.succeed = UNKNOWN


        self.new_goal = False
        self.goal_angle = 0.0
        self.goal_distance = MAX_GOAL_DISTANCE
        self.initial_distance_to_goal = MAX_GOAL_DISTANCE

        self.scan_ranges = [REAL_LIDAR_DISTANCE_CAP] * REAL_N_SCAN_SAMPLES
        self.obstacle_distance = REAL_LIDAR_DISTANCE_CAP

        self.difficulty_radius = 1
        self.local_step = 0

        # ________________________ Version (1)____________________________


        self.scan_ranges = []
        self.obstacle_distance = float('inf')
        self.prev_distance = None
        self.min_history = collections.deque(maxlen=3)  # for median filtering


        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)
        # publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.velo_topic, qos)
        # subscribers
        self.goal_pose_sub = self.create_subscription(Pose, self.goal_topic, self.goal_pose_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile=qos_profile_sensor_data)
        # 9/9/2025 added chagpt
        #self.scan_pub = self.create_publisher(LaserScan, '/scan_corrected', qos_profile=qos_profile_sensor_data)
        # servers
        self.step_comm_server = self.create_service(DrlStep, 'step_comm', self.step_comm_callback)
        self.goal_comm_server = self.create_service(Goal, 'goal_comm', self.goal_comm_callback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""

    def goal_pose_callback(self, msg):
        self.goal_x = msg.position.x
        self.goal_y = msg.position.y
        self.new_goal = True
        print(f"new goal! x: {self.goal_x} y: {self.goal_y}")

    def goal_comm_callback(self, request, response):
        response.new_goal = self.new_goal
        return response

    def odom_callback(self, msg):
        # self.robot_x = msg.pose.pose.position.x * -1
        # self.robot_y = msg.pose.pose.position.y * -1
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, _, self.robot_heading = util.euler_from_quaternion(msg.pose.pose.orientation)
        self.robot_tilt = msg.pose.pose.orientation.y

        # calculate traveled distance for logging
        if self.local_step % 32 == 0:
            self.total_distance += math.sqrt(
                (self.robot_x_prev - self.robot_x)**2 +
                (self.robot_y_prev - self.robot_y)**2)
            self.robot_x_prev = self.robot_x
            self.robot_y_prev = self.robot_y

        diff_y = self.goal_y - self.robot_y
        diff_x = self.goal_x - self.robot_x
        distance_to_goal = math.sqrt(diff_x**2 + diff_y**2)
        heading_to_goal = math.atan2(diff_y, diff_x)
        goal_angle = heading_to_goal - self.robot_heading

        while goal_angle > math.pi:
            goal_angle -= 2 * math.pi
        while goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = distance_to_goal
        self.goal_angle = goal_angle

    # def scan_callback(self, msg):
    #     if len(msg.ranges) != REAL_N_SCAN_SAMPLES:
    #         print(f"more or less scans than expected! check model.sdf, got: {len(msg.ranges)}, expected: {REAL_N_SCAN_SAMPLES}")
    #     # normalize laser values
    #     self.obstacle_distance = 1
    #     for i in range(REAL_N_SCAN_SAMPLES):
    #             self.scan_ranges[i] = numpy.clip(float(msg.ranges[i] - REAL_LIDAR_CORRECTION) / REAL_LIDAR_DISTANCE_CAP, 0, 1)
    #             if self.scan_ranges[i] < self.obstacle_distance:
    #                 self.obstacle_distance = self.scan_ranges[i]
    #     self.obstacle_distance *= REAL_LIDAR_DISTANCE_CAP

    # def scan_callback(self, msg):
    #     # Downsample real LiDAR scan to match expected samples
    #     full_scan = numpy.array(msg.ranges)

    #     # Replace NaN/Inf with max range
    #     full_scan = numpy.nan_to_num(full_scan, nan=msg.range_max, posinf=msg.range_max, neginf=0.0)

    #     # Downsample evenly to expected number
    #     if len(full_scan) != REAL_N_SCAN_SAMPLES:
    #         indices = numpy.linspace(0, len(full_scan) - 1, REAL_N_SCAN_SAMPLES, dtype=int)
    #         scan_data = full_scan[indices]
    #     else:
    #         scan_data = full_scan

    #     # Normalize laser values
    #     self.obstacle_distance = 1
    #     for i in range(REAL_N_SCAN_SAMPLES):
    #         self.scan_ranges[i] = numpy.clip(
    #             float(scan_data[i] - REAL_LIDAR_CORRECTION) / REAL_LIDAR_DISTANCE_CAP,
    #             0, 1
    #         )
    #         if self.scan_ranges[i] < self.obstacle_distance:
    #             self.obstacle_distance = self.scan_ranges[i]

    #     self.obstacle_distance *= REAL_LIDAR_DISTANCE_CAP

    # 66666666666666666666666666666666
    # def scan_callback(self, msg):

    #     expected = REAL_N_SCAN_SAMPLES

    #     ranges = np.array(msg.ranges, dtype=float)

    #     # Replace NaN / Inf with max range
    #     rng_max = msg.range_max if msg.range_max > 0 else REAL_LIDAR_DISTANCE_CAP
    #     rng_min = msg.range_min if msg.range_min > 0 else 0.0
    #     ranges = np.nan_to_num(ranges, nan=rng_max, posinf=rng_max, neginf=rng_max)

    #     # Treat anything below min range as "no return" → set to max
    #     ranges[ranges < rng_min] = rng_max

    #     # Downsample if needed
    #     if ranges.size != expected:
    #         idx = np.linspace(0, ranges.size - 1, expected, dtype=int)
    #         ranges = ranges[idx]

    #     # Apply correction and cap
    #     corrected = np.clip(ranges - REAL_LIDAR_CORRECTION, 0.0, REAL_LIDAR_DISTANCE_CAP)

    #     # Normalize for the network
    #     self.scan_ranges = list(np.clip(corrected / REAL_LIDAR_DISTANCE_CAP, 0.0, 1.0))

    #     # Obstacle distance in meters
    #     self.obstacle_distance = float(corrected.min())

    # geminiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii


# Assuming your constants are defined elsewhere
# REAL_N_SCAN_SAMPLES, REAL_LIDAR_CORRECTION, REAL_LIDAR_DISTANCE_CAP, etc.

    # def scan_callback(self, msg):
    #     expected = REAL_N_SCAN_SAMPLES
    #     ranges = np.array(msg.ranges, dtype=float)

    #     # 1. Get LiDAR's physical limits, with safe fallbacks
    #     rng_max = msg.range_max if msg.range_max > 0 else REAL_LIDAR_DISTANCE_CAP
    #     rng_min = msg.range_min if msg.range_min > 0 else 0.1 # Use a sensible default like 10cm

    #     # 2. Create a clean copy of ranges for the collision checker (high-resolution)
    #     # Replace NaN/Inf with max range, but leave 0.0s for now.
    #     safety_ranges = np.nan_to_num(ranges, nan=rng_max, posinf=rng_max, neginf=rng_max)

    #     # 3. ✅ CRITICAL FIX: To find the true minimum obstacle distance, we should only
    #     # consider valid returns that are above the sensor's minimum range.
    #     # Create a mask of only the valid readings.
    #     valid_mask = safety_ranges > rng_min
        
    #     if np.any(valid_mask):
    #         # If there are any valid readings, find the minimum among them.
    #         self.obstacle_distance = np.min(safety_ranges[valid_mask])
    #     else:
    #         # If all readings are invalid (e.g., all zeros), assume a safe distance.
    #         self.obstacle_distance = rng_max

    #     # Apply the systematic correction to the distance value.
    #     self.obstacle_distance = max(0.0, self.obstacle_distance - REAL_LIDAR_CORRECTION)

    #     # --- Now, prepare the scan for the neural network ---
        
    #     # 4. For the network input, we can now handle the 0.0s.
    #     # Replace any 0.0 or sub-min values with the max range so the network sees a clear path.
    #     network_ranges = ranges 
    #     network_ranges[network_ranges <= rng_min] = rng_max # Treat invalid/too-close as "clear" for the NN input
    #     network_ranges = np.nan_to_num(network_ranges, nan=rng_max, posinf=rng_max, neginf=rng_max)

    #     # 5. Apply correction & cap for the network scan
    #     corrected_network_ranges = np.clip(network_ranges - REAL_LIDAR_CORRECTION, 0.0, REAL_LIDAR_DISTANCE_CAP)

    #     # 6. Perform safe downsampling for the network
    #     downsampled_ranges = corrected_network_ranges
    #     if downsampled_ranges.size != expected:
    #         if downsampled_ranges.size % expected == 0:
    #             downsampled_ranges = np.min(downsampled_ranges.reshape(-1, downsampled_ranges.size // expected), axis=1)
    #         else:
    #             idx = np.linspace(0, downsampled_ranges.size - 1, expected, dtype=int)
    #             downsampled_ranges = downsampled_ranges[idx]

    #     # 7. Normalize and set the final scan for the network
    #     self.scan_ranges = list(np.clip(downsampled_ranges / REAL_LIDAR_DISTANCE_CAP, 0.0, 1.0))

    # def scan_callback(self, msg):
    #     expected = REAL_N_SCAN_SAMPLES
    #     ranges = np.array(msg.ranges, dtype=float)

    #     # 1. Get LiDAR's physical limits, with safe fallbacks
    #     rng_max = msg.range_max if msg.range_max > 0 else REAL_LIDAR_DISTANCE_CAP
    #     rng_min = msg.range_min if msg.range_min > 0 else 0.1  # sensible default

    #     # 2. Create a clean copy of ranges for the collision checker (high-resolution)
    #     safety_ranges = np.nan_to_num(ranges, nan=rng_max, posinf=rng_max, neginf=rng_max)

    #     # 3. Minimum obstacle distance from valid readings
    #     valid_mask = safety_ranges > rng_min
    #     if np.any(valid_mask):
    #         self.obstacle_distance = np.min(safety_ranges[valid_mask])
    #     else:
    #         self.obstacle_distance = rng_max

    #     # Apply correction
    #     self.obstacle_distance = max(0.0, self.obstacle_distance - REAL_LIDAR_CORRECTION)

    #     # --- Now, prepare the scan for the neural network ---
    #     # 4. Replace invalid / too-close values
    #     network_ranges = ranges
    #     network_ranges[network_ranges <= rng_min] = rng_max
    #     network_ranges = np.nan_to_num(network_ranges, nan=rng_max, posinf=rng_max, neginf=rng_max)

    #     # 5. Apply correction & cap
    #     corrected_network_ranges = np.clip(network_ranges - REAL_LIDAR_CORRECTION, 0.0, REAL_LIDAR_DISTANCE_CAP)

    #     # 6. Pad Hokuyo 240° to 360° with max-range values
    #     if corrected_network_ranges.size < 360:
    #         pad_left = (360 - corrected_network_ranges.size) // 2
    #         pad_right = 360 - corrected_network_ranges.size - pad_left
    #         corrected_network_ranges = np.pad(corrected_network_ranges, (pad_left, pad_right), 
    #                                         mode='constant', constant_values=rng_max)

    #     # 7. Downsample to expected size
    #     downsampled_ranges = corrected_network_ranges
    #     if downsampled_ranges.size != expected:
    #         if downsampled_ranges.size % expected == 0:
    #             downsampled_ranges = np.min(
    #                 downsampled_ranges.reshape(-1, downsampled_ranges.size // expected), axis=1
    #             )
    #         else:
    #             idx = np.linspace(0, downsampled_ranges.size - 1, expected, dtype=int)
    #             downsampled_ranges = downsampled_ranges[idx]

    #     # 8. Final cleaning (NaN/Inf/out-of-range handling)
    #     self.scan_ranges = [
    #         float('inf') if (x is None or np.isnan(x) or np.isinf(x) or x < rng_min or x > REAL_LIDAR_DISTANCE_CAP) else x
    #         for x in downsampled_ranges
    #     ]

    #     # 9. Normalize to [0,1] for the NN
    #     self.scan_ranges = list(np.clip(np.array(self.scan_ranges) / REAL_LIDAR_DISTANCE_CAP, 0.0, 1.0))


        
    # Optional: For debugging, print the true minimum vs what the network sees
    # print(f"Real Min Distance: {self.obstacle_distance:.2f}m")


    # hinttttttttttttttttttttttttttttttttttttttttttttttttttttt
    # def scan_callback(self, msg):
    #     if len(msg.ranges) > REAL_N_SCAN_SAMPLES:
    #         step = len(msg.ranges) // REAL_N_SCAN_SAMPLES
    #         self.scan_ranges = msg.ranges[::step][:REAL_N_SCAN_SAMPLES]
    #     else:
    #         self.scan_ranges = list(msg.ranges)

    #     # Handle NaN / Inf and out-of-range values
    #     self.scan_ranges = [
    #         float('inf') if (x is None or numpy.isnan(x) or numpy.isinf(x) or x < 0.11 or x > 3) else x
    #         for x in self.scan_ranges
    #     ]

    #     ## normalize laser values
    #     self.obstacle_distance = 1
    #     for i in range(REAL_N_SCAN_SAMPLES):
    #         self.scan_ranges[i] = numpy.clip(
    #             (self.scan_ranges[i] - REAL_LIDAR_CORRECTION) / REAL_LIDAR_DISTANCE_CAP,
    #             0, 1
    #         )
    #         if self.scan_ranges[i] < self.obstacle_distance:
    #             self.obstacle_distance = self.scan_ranges[i]

    #     self.obstacle_distance *= REAL_LIDAR_DISTANCE_CAP

    # _______________________________ 25/08/ 2025 -------- Version(1) _______________________________________________ 

    # def scan_callback(self, msg):
    #     # if len(msg.ranges) > REAL_N_SCAN_SAMPLES:
    #     #     step = len(msg.ranges) // REAL_N_SCAN_SAMPLES
    #     #     self.scan_ranges = msg.ranges[::step][:REAL_N_SCAN_SAMPLES]
    #     # else:
    #     #     self.scan_ranges = list(msg.ranges)

    #     # # Handle NaN / Inf and out-of-range values
    #     # rng_min = msg.range_min if msg.range_min > 0 else 0.05  # safe default
    #     # rng_max = msg.range_max if msg.range_max > 0 else REAL_LIDAR_DISTANCE_CAP
    #     # self.scan_ranges = [
    #     #     float('inf') if (x is None or numpy.isnan(x) or numpy.isinf(x) or x < rng_min or x > rng_max) else x
    #     #     for x in self.scan_ranges
    #     # ]

    #     if len(msg.ranges) >= REAL_N_SCAN_SAMPLES:
    #         step = len(msg.ranges) // REAL_N_SCAN_SAMPLES
    #         self.scan_ranges = msg.ranges[::step][:REAL_N_SCAN_SAMPLES]
    #     else:
    #         # Pad or interpolate to get exactly REAL_N_SCAN_SAMPLES
    #         import numpy as np
    #         if len(msg.ranges) == 0:
    #             self.scan_ranges = [float('inf')] * REAL_N_SCAN_SAMPLES
    #         else:
    #             self.scan_ranges = list(
    #                 np.interp(
    #                     np.linspace(0, len(msg.ranges) - 1, REAL_N_SCAN_SAMPLES),
    #                     np.arange(len(msg.ranges)),
    #                     msg.ranges
    #                 )
    #             )


    #     # Normalize laser values
    #     self.obstacle_distance = 1
    #     for i in range(REAL_N_SCAN_SAMPLES):
    #         self.scan_ranges[i] = numpy.clip(
    #             (self.scan_ranges[i] - REAL_LIDAR_CORRECTION) / REAL_LIDAR_DISTANCE_CAP,
    #             0, 1
    #         )
    #         if self.scan_ranges[i] < self.obstacle_distance:
    #             self.obstacle_distance = self.scan_ranges[i]

    #     # Scale back to meters
    #     self.obstacle_distance *= REAL_LIDAR_DISTANCE_CAP

    #     # --- Filtering ---
    #     # 1) median filter
    #     self.min_history.append(self.obstacle_distance)
    #     filtered = float(numpy.median(self.min_history))

    #     # 2) reject sudden jumps (spike noise)
    #     if self.prev_distance is not None:
    #         if filtered < 0.2 and abs(filtered - self.prev_distance) > 0.3:
    #             # ignore noise, keep previous
    #             filtered = self.prev_distance

    #     self.prev_distance = filtered
    #     self.obstacle_distance = filtered

    # _______________________________ 25/08/ 2025 -------- Version(2) _______________________________________________ 


    # def scan_callback(self, msg):
    #     if len(msg.ranges) > REAL_N_SCAN_SAMPLES:
    #         step = len(msg.ranges) // REAL_N_SCAN_SAMPLES
    #         self.scan_ranges = msg.ranges[::step][:REAL_N_SCAN_SAMPLES]
    #     else:
    #         self.scan_ranges = list(msg.ranges)

    #     # Handle NaN / Inf and out-of-range values
    #     rng_min = msg.range_min if msg.range_min > 0 else 0.05  # safe default
    #     rng_max = msg.range_max if msg.range_max > 0 else REAL_LIDAR_DISTANCE_CAP
    #     self.scan_ranges = [
    #         float('inf') if (x is None or np.isnan(x) or np.isinf(x) or x < rng_min or x > rng_max) else x
    #         for x in self.scan_ranges
    #     ]

    #     # Normalize laser values
    #     self.obstacle_distance = 1
    #     for i in range(REAL_N_SCAN_SAMPLES):
    #         self.scan_ranges[i] = np.clip(
    #             (self.scan_ranges[i] - REAL_LIDAR_CORRECTION) / REAL_LIDAR_DISTANCE_CAP,
    #             0, 1
    #         )
    #         if self.scan_ranges[i] < self.obstacle_distance:
    #             self.obstacle_distance = self.scan_ranges[i]

    #     # Scale back to meters
    #     self.obstacle_distance *= REAL_LIDAR_DISTANCE_CAP

    #     # --- Filtering ---
    #     # 1) median filter
    #     self.min_history.append(self.obstacle_distance)
    #     filtered = float(np.median(self.min_history))

    #     # 2) reject sudden jumps (spike noise)
    #     if self.prev_distance is not None:
    #         if filtered < 0.2 and abs(filtered - self.prev_distance) > 0.3:
    #             # ignore noise, keep previous
    #             filtered = self.prev_distance

    #     self.prev_distance = filtered
    #     self.obstacle_distance = filtered

    #     # --- 360° padding for network ---
    #     corrected_network_ranges = np.array(self.scan_ranges) * REAL_LIDAR_DISTANCE_CAP  # back to meters
    #     if corrected_network_ranges.size < 360:
    #         pad_left = (360 - corrected_network_ranges.size) // 2
    #         pad_right = 360 - corrected_network_ranges.size - pad_left
    #         corrected_network_ranges = np.pad(
    #             corrected_network_ranges,
    #             (pad_left, pad_right),
    #             mode='constant',
    #             constant_values=rng_max
    #         )
    #     self.corrected_network_ranges = corrected_network_ranges



    # def scan_callback(self, msg):
    #     # --- Downsample or interpolate to REAL_N_SCAN_SAMPLES ---
    #     if len(msg.ranges) >= REAL_N_SCAN_SAMPLES:
    #         step = len(msg.ranges) // REAL_N_SCAN_SAMPLES
    #         self.scan_ranges = msg.ranges[::step][:REAL_N_SCAN_SAMPLES]
    #     else:
    #         if len(msg.ranges) == 0:
    #             self.scan_ranges = [float('inf')] * REAL_N_SCAN_SAMPLES
    #         else:
    #             self.scan_ranges = list(
    #                 np.interp(
    #                     np.linspace(0, len(msg.ranges) - 1, REAL_N_SCAN_SAMPLES),
    #                     np.arange(len(msg.ranges)),
    #                     msg.ranges
    #                 )
    #             )

    #     # --- Apply correction + normalize for internal use ---
    #     self.obstacle_distance = 1
    #     for i in range(REAL_N_SCAN_SAMPLES):
    #         self.scan_ranges[i] = np.clip(
    #             (self.scan_ranges[i] - REAL_LIDAR_CORRECTION) / REAL_LIDAR_DISTANCE_CAP,
    #             0, 1
    #         )
    #         if self.scan_ranges[i] < self.obstacle_distance:
    #             self.obstacle_distance = self.scan_ranges[i]

    #     # Scale back to meters
    #     self.obstacle_distance *= REAL_LIDAR_DISTANCE_CAP

    #     # --- Filtering ---
    #     self.min_history.append(self.obstacle_distance)
    #     filtered = float(np.median(self.min_history))

    #     if self.prev_distance is not None:
    #         if filtered < 0.2 and abs(filtered - self.prev_distance) > 0.3:
    #             filtered = self.prev_distance

    #     self.prev_distance = filtered
    #     self.obstacle_distance = filtered

    #     # --- Republish corrected scan for RViz2 ---
    #     corrected_msg = LaserScan()
    #     corrected_msg.header = msg.header
    #     corrected_msg.angle_min = msg.angle_min
    #     corrected_msg.angle_max = msg.angle_max
    #     corrected_msg.angle_increment = msg.angle_increment
    #     corrected_msg.time_increment = msg.time_increment
    #     corrected_msg.scan_time = msg.scan_time
    #     corrected_msg.range_min = msg.range_min
    #     corrected_msg.range_max = msg.range_max

    #     corrected_ranges = []
    #     for r in msg.ranges:
    #         if not math.isinf(r) and not math.isnan(r):
    #             corrected_ranges.append(max(0.0, r - REAL_LIDAR_CORRECTION))
    #         else:
    #             corrected_ranges.append(r)

    #     corrected_msg.ranges = corrected_ranges
    #     corrected_msg.intensities = msg.intensities

    #     self.scan_pub.publish(corrected_msg)
# -----------------------------------------------------------------------------------------------------
    # def scan_callback(self, msg):
    #    if len(msg.ranges) > REAL_N_SCAN_SAMPLES:			
   	#     self.scan_ranges=msg.ranges[::int(len(msg.ranges)/REAL_N_SCAN_SAMPLES)] 
    #    if len(self.scan_ranges) != REAL_N_SCAN_SAMPLES:
    #        print(f"more or less scans than expected! check model.sdf, got: {len(msg.ranges)}, expected: {REAL_N_SCAN_SAMPLES}")
    #    self.scan_ranges = [float('inf') if x<0.05 else x for x in self.scan_ranges]	
    #    ## normalize laser values
    #    self.obstacle_distance = 1
    #    for i in range(REAL_N_SCAN_SAMPLES):
    #            self.scan_ranges[i] = numpy.clip(float(self.scan_ranges[i] - REAL_LIDAR_CORRECTION) / REAL_LIDAR_DISTANCE_CAP, 0, 1)
    #            if self.scan_ranges[i] < self.obstacle_distance:
    #                self.obstacle_distance = self.scan_ranges[i]
    #    self.obstacle_distance *= REAL_LIDAR_DISTANCE_CAP

    # def scan_callback(self, msg):
    #     indices = np.linspace(0, len(msg.ranges) - 1, REAL_N_SCAN_SAMPLES, dtype=int)
    #     sampled_ranges = [msg.ranges[i] for i in indices]

    #     self.scan_ranges = []
    #     self.obstacle_distance = 1.0  # normalized

    #    #print("Raw sampled ranges:", [round(r, 2) for r in sampled_ranges[:10]])

    #     for r in sampled_ranges:
    #         if r <= 0.05 or np.isinf(r) or np.isnan(r):
    #             r = REAL_LIDAR_DISTANCE_CAP   # max range

    #         # Normalize: 0 (near) → 1 (far)
    #         val = np.clip(r / REAL_LIDAR_DISTANCE_CAP, 0, 1)

    #         #print(f"Raw: {r:.2f} → Norm: {val:.2f}")

    #         self.scan_ranges.append(val)

    #         if val < self.obstacle_distance:
    #             self.obstacle_distance = val

    #     self.obstacle_distance *= REAL_LIDAR_DISTANCE_CAP

    #     # print("Final normalized scan sample:", np.round(self.scan_ranges[:10], 2))
    #     # print("MinD (m):", round(self.obstacle_distance, 2))


    def scan_callback(self, msg):
        # Evenly sample REAL_N_SCAN_SAMPLES beams
        indices = np.linspace(0, len(msg.ranges) - 1, REAL_N_SCAN_SAMPLES, dtype=int)
        sampled_ranges = np.array([msg.ranges[i] for i in indices], dtype=np.float32)

        # Replace invalid values (NaN, inf, too close)
        sampled_ranges = np.nan_to_num(sampled_ranges, nan=REAL_LIDAR_DISTANCE_CAP, 
                                    posinf=REAL_LIDAR_DISTANCE_CAP, neginf=REAL_LIDAR_DISTANCE_CAP)
        sampled_ranges[sampled_ranges < msg.range_min] = REAL_LIDAR_DISTANCE_CAP

        # Optional: median filter to reduce spikes
        if len(sampled_ranges) >= 3:
            sampled_ranges = np.convolve(sampled_ranges, [1/3, 1/3, 1/3], mode='same')

        self.scan_ranges = []
        self.obstacle_distance = 1.0  # normalized

        for r in sampled_ranges:
            # Clip to cap range
            r = min(r, REAL_LIDAR_DISTANCE_CAP)

            # Normalize: 0 = very close obstacle, 1 = far/clear
            val = np.clip(r / REAL_LIDAR_DISTANCE_CAP, 0, 1)

            self.scan_ranges.append(val)

            # Track closest obstacle (normalized)
            if val < self.obstacle_distance:
                self.obstacle_distance = val

        # Convert back obstacle_distance into meters
        self.obstacle_distance *= REAL_LIDAR_DISTANCE_CAP








    




    def stop_reset_robot(self, success):
        self.cmd_vel_pub.publish(Twist()) # stop robot
        self.done = True


    def get_state(self, action_linear_previous, action_angular_previous):
        state = copy.deepcopy(self.scan_ranges)                                             # range: [ 0, 1]
        state.append(float(numpy.clip((self.goal_distance / MAX_GOAL_DISTANCE), 0, 1)))     # range: [ 0, 1]
        state.append(float(self.goal_angle) / math.pi)                                      # range: [-1, 1]
        state.append(float(action_linear_previous))                                         # range: [-1, 1]
        state.append(float(action_angular_previous))                                        # range: [-1, 1]
        self.local_step += 1

        if self.local_step <= 15: # Grace period to wait for fresh sensor input
            return state
        # Success
        if self.goal_distance < REAL_THRESHOLD_GOAL:
            print("Outcome: Goal reached! :)")
            self.succeed = SUCCESS
        # Collision
        elif self.obstacle_distance < REAL_THRESHOLD_COLLISION:
            print("Collision! (wall) :(")
            self.succeed = COLLISION_WALL
        # Timeout
        # elif self.time_sec >= self.episode_deadline:
        #     print("Outcome: Time out! :(")
        #     self.succeed = TIMEOUT
        if self.succeed is not UNKNOWN:
            self.stop_reset_robot(self.succeed == SUCCESS)
        return state

    def initalize_episode(self, response):
        self.initial_distance_to_goal = self.goal_distance
        response.state = self.get_state(0, 0)
        response.reward = 0.0
        response.done = False
        response.distance_traveled = 0.0
        return response

    # orignal ---------------------------------------
    def step_comm_callback(self, request, response):
        if len(request.action) == 0:
            return self.initalize_episode(response)

        # Unnormalize actions
        if ENABLE_BACKWARD:
            action_linear = request.action[LINEAR] * REAL_SPEED_LINEAR_MAX
        else:
            action_linear = (request.action[LINEAR] + 1) / 2 * REAL_SPEED_LINEAR_MAX
        action_angular = request.action[ANGULAR] * REAL_SPEED_ANGULAR_MAX

        # Publish action cmd
        twist = Twist()
        twist.linear.x = action_linear
        twist.angular.z = action_angular
        self.cmd_vel_pub.publish(twist)

        # Prepare repsonse
        response.state = self.get_state(request.previous_action[LINEAR], request.previous_action[ANGULAR])
        #response.reward = 0.0

        # Bug: Compute reward
        response.reward = rw.get_reward(self.succeed, action_linear, action_angular, self.goal_distance,
                                    self.goal_angle, self.obstacle_distance)
        response.done = self.done
        response.success = self.succeed
        response.distance_traveled = 0.0
        if self.done:
            self.new_goal = False
            response.distance_traveled = self.total_distance
            # Reset variables
            self.succeed = UNKNOWN
            self.total_distance = 0.0
            self.local_step = 0
            self.done = False
        if self.local_step % 10 == 0:
            print(f"Rtot: {response.reward:<8.2f}GD: {self.goal_distance:<8.2f}GA: {math.degrees(self.goal_angle):.1f}°\t", end='')
            print(f"MinD: {self.obstacle_distance:<8.2f}Alin: {request.action[LINEAR]:<7.1f}Aturn: {request.action[ANGULAR]:<7.1f}")
        return response
    # def step_comm_callback(self, request, response):
    #     if len(request.action) == 0:
    #         return self.initalize_episode(response)

    #     # Unnormalize actions from the AI agent
    #     if ENABLE_BACKWARD:
    #         action_linear = request.action[LINEAR] * REAL_SPEED_LINEAR_MAX
    #     else:
    #         action_linear = (request.action[LINEAR] + 1) / 2 * REAL_SPEED_LINEAR_MAX
    #     action_angular = request.action[ANGULAR] * REAL_SPEED_ANGULAR_MAX

    #     # =================================================================
    #     # ✅ START: THIS IS THE CORRECT PLACE FOR THE SAFETY OVERRIDE
    #     # We check for obstacles BEFORE sending any command to the motors.
    #     # =================================================================
        
    #     SAFE_THRESHOLD = 0.35 # Use a safe value like 35cm for testing
        
    #     final_linear_vel = action_linear
    #     final_angular_vel = action_angular
        
    #     if self.obstacle_distance < SAFE_THRESHOLD:
    #         # If an obstacle is inside the safety bubble, override the AI's action.
    #         print(f"!!! EMERGENCY STOP !!! Obstacle at {self.obstacle_distance:.2f}m is inside the {SAFE_THRESHOLD}m safety zone.")
    #         final_linear_vel = 0.0
    #         final_angular_vel = 0.0 # A full stop is the safest test.

    #     # =================================================================
    #     # ✅ END: SAFETY OVERRIDE BLOCK
    #     # =================================================================

    #     # Publish the FINAL, safety-checked action cmd
    #     twist = Twist()
    #     twist.linear.x = final_linear_vel   # Use the safety-checked value
    #     twist.angular.z = final_angular_vel # Use the safety-checked value
    #     self.cmd_vel_pub.publish(twist)

    #     # Prepare response AFTER the action has been taken
    #     response.state = self.get_state(request.previous_action[LINEAR], request.previous_action[ANGULAR])
    #     response.reward = 0.0
    #     response.done = self.done
    #     response.success = self.succeed
    #     response.distance_traveled = 0.0
    #     if self.done:
    #         self.new_goal = False
    #         response.distance_traveled = self.total_distance
    #         # Reset variables
    #         self.succeed = UNKNOWN
    #         self.total_distance = 0.0
    #         self.local_step = 0
    #         self.done = False
        
    #     # This print statement will now reflect the safety override
    #     if self.local_step % 10 == 0:
    #         print(f"Rtot: {response.reward:<8.2f}GD: {self.goal_distance:<8.2f}GA: {math.degrees(self.goal_angle):.1f}°\t", end='')
    #         print(f"MinD: {self.obstacle_distance:<8.2f}Alin: {request.action[LINEAR]:<7.1f}Aturn: {request.action[ANGULAR]:<7.1f}")
        
    #     return response



def main(args=sys.argv[1:]):
    rclpy.init(args=args)
    if len(args) == 0:
        drl_environment = DRLEnvironment()
    else:
        rclpy.shutdown()
        quit("ERROR: wrong number of arguments!")
    rclpy.spin(drl_environment)
    drl_environment.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
