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

# _____________________ imports block ___________________________________________
import math
import sys
import copy
import collections
import numpy as np

from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from turtlebot3_msgs.srv import DrlStep, Goal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

# keep reward import and settings import as-is
from . import reward as rw

from ..common import utilities as util
from ..common.settings import ENABLE_BACKWARD, UNKNOWN, SUCCESS, COLLISION_WALL, REAL_ARENA_LENGTH, REAL_ARENA_WIDTH, \
    REAL_SPEED_LINEAR_MAX, REAL_SPEED_ANGULAR_MAX, REAL_N_SCAN_SAMPLES, REAL_LIDAR_DISTANCE_CAP, REAL_LIDAR_CORRECTION, \
    REAL_THRESHOLD_COLLISION, REAL_THRESHOLD_GOAL, REAL_TOPIC_SCAN, REAL_TOPIC_VELO, REAL_TOPIC_ODOM


# _______________________________________________constants________________________________________
LINEAR = 0
ANGULAR = 1

MAX_GOAL_DISTANCE = math.sqrt(REAL_ARENA_LENGTH**2 + REAL_ARENA_WIDTH**2)


# _____________________________________________DRLEnvironment Class________________________________________
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

        # _________________________ LiDAR scan and obstacle distance ________________________
        # inside __init__ after self.goal/topic initializations
        # Default safe scan vector (normalized: 1.0 = clear)
        self.scan_ranges = [1.0] * REAL_N_SCAN_SAMPLES

        # obstacle distances in meters (explicit)
        self.obstacle_distance_front = REAL_LIDAR_DISTANCE_CAP
        self.obstacle_distance_rear = REAL_LIDAR_DISTANCE_CAP
        self.obstacle_distance_any = REAL_LIDAR_DISTANCE_CAP    # global min (any direction)
        # keep self.obstacle_distance for backward compatibility (meters)
        self.obstacle_distance = REAL_LIDAR_DISTANCE_CAP

        # for filtering
        self.prev_distance = None
        self.min_history = collections.deque(maxlen=3)    # optional smoothing

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

    def _downsample_by_min(self, arr: np.ndarray, out_size: int, max_val: float):
        """
        Split arr into out_size bins and take min in each bin (preserves obstacles).
        If arr is smaller than out_size, do linear interpolation.
        """
        if arr.size == out_size:
            return arr.copy()
        if arr.size > out_size:
            bins = np.array_split(arr, out_size)
            return np.array([np.min(b) if b.size > 0 else max_val for b in bins], dtype=np.float32)
        # arr.size < out_size -> interpolate smoothly
        x_old = np.linspace(0, 1, arr.size)
        x_new = np.linspace(0, 1, out_size)
        return np.interp(x_new, x_old, arr).astype(np.float32)

    def _sliding_median(self, arr: np.ndarray, k=3):
        # simple median filter with small kernel k (odd)
        n = arr.size
        if n == 0:
            return arr
        pad = np.pad(arr, (k//2, k//2), mode='edge')
        out = np.zeros_like(arr)
        for i in range(n):
            out[i] = np.median(pad[i:i+k])
        return out

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

    def scan_callback(self, msg: LaserScan):
        # convert to array
        ranges = np.array(msg.ranges, dtype=np.float32)

        # sensor limits (safe fallbacks)
        rng_max = msg.range_max if (hasattr(msg, 'range_max') and msg.range_max and msg.range_max > 0) else REAL_LIDAR_DISTANCE_CAP
        rng_min = msg.range_min if (hasattr(msg, 'range_min') and msg.range_min and msg.range_min > 0) else 0.05

        # replace NaN/Inf
        ranges = np.nan_to_num(ranges, nan=rng_max, posinf=rng_max, neginf=rng_max)

        # treat too-close or zero as no-return -> set to max
        ranges[ranges < rng_min] = rng_max

        # apply systematic correction and cap (meters)
        corrected = np.clip(ranges - REAL_LIDAR_CORRECTION, 0.0, REAL_LIDAR_DISTANCE_CAP)

        # global min (any direction)
        if corrected.size > 0:
            global_min = float(corrected.min())
        else:
            global_min = float(rng_max)

        # angles for full scan (use original resolution for sector checks)
        if hasattr(msg, 'angle_min') and hasattr(msg, 'angle_increment'):
            angles = (msg.angle_min + np.arange(ranges.size) * msg.angle_increment).astype(np.float32)
            # normalize to [-pi, pi]
            ang_norm = (angles + np.pi) % (2 * np.pi) - np.pi
            front_deg = 25  # front sector +/- 25 degrees
            front_mask = np.abs(ang_norm) <= np.deg2rad(front_deg)
            if np.any(front_mask):
                front_min = float(corrected[front_mask].min())
            else:
                front_min = global_min
            # rear sector (approx)
            rear_mask = np.abs(ang_norm) >= (np.pi - np.deg2rad(front_deg))
            if np.any(rear_mask):
                rear_min = float(corrected[rear_mask].min())
            else:
                rear_min = global_min
        else:
            front_min = global_min
            rear_min = global_min

        # Downsample to expected size but preserve obstacles -> take min in each bin
        down = self._downsample_by_min(corrected, REAL_N_SCAN_SAMPLES, rng_max)

        # Median filter to remove spikes
        down = self._sliding_median(down, k=3)

        # Normalize for NN (0..1)
        norm = np.clip(down / REAL_LIDAR_DISTANCE_CAP, 0.0, 1.0)
        self.scan_ranges = list(norm.astype(np.float32))

        # store obstacle distances (meters)
        self.obstacle_distance_any = global_min
        self.obstacle_distance_front = front_min
        self.obstacle_distance_rear = rear_min
        # default obstacle_distance used elsewhere — keep it as front min (meters)
        self.obstacle_distance = self.obstacle_distance_front

        # small smoothing history if you still want it
        self.min_history.append(self.obstacle_distance)
        filtered = float(np.median(self.min_history))
        # Reject unrealistic spikes: if jump too large keep previous
        if self.prev_distance is not None and abs(filtered - self.prev_distance) > 0.6:
            filtered = self.prev_distance
        self.prev_distance = filtered
        self.obstacle_distance = filtered




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

    def step_comm_callback(self, request, response):
        if len(request.action) == 0:
            return self.initalize_episode(response)  # keep existing name to avoid refactor

        # Unnormalize actions
        if ENABLE_BACKWARD:
            action_linear = float(request.action[LINEAR]) * REAL_SPEED_LINEAR_MAX
        else:
            action_linear = float((request.action[LINEAR] + 1.0) / 2.0 * REAL_SPEED_LINEAR_MAX)
        action_angular = float(request.action[ANGULAR]) * REAL_SPEED_ANGULAR_MAX

        # Safety: clip to permitted ranges
        action_linear = float(np.clip(action_linear, -REAL_SPEED_LINEAR_MAX, REAL_SPEED_LINEAR_MAX))
        action_angular = float(np.clip(action_angular, -REAL_SPEED_ANGULAR_MAX, REAL_SPEED_ANGULAR_MAX))

        # Safety policy (front/back aware)
        SAFE_FRONT_STOP = max(REAL_THRESHOLD_COLLISION + 0.05, 0.15)  # tweakable
        SAFE_REAR_STOP = SAFE_FRONT_STOP

        final_linear = action_linear
        final_angular = action_angular

        # If trying to move forward but front obstacle is too close -> override forward to 0 (allow backing)
        if action_linear > 0.0 and self.obstacle_distance_front < SAFE_FRONT_STOP:
            self.get_logger().warn(f"Front obstacle {self.obstacle_distance_front:.2f}m < {SAFE_FRONT_STOP:.2f}m -> stopping forward motion.")
            final_linear = 0.0
            # you can optionally set final_linear = -0.02 to force small reverse instead of full stop

        # If trying to move backward but rear obstacle is too close -> block backwards
        if action_linear < 0.0 and self.obstacle_distance_rear < SAFE_REAR_STOP:
            self.get_logger().warn(f"Rear obstacle {self.obstacle_distance_rear:.2f}m < {SAFE_REAR_STOP:.2f}m -> blocking backward motion.")
            final_linear = 0.0

        # Publish SAFE command
        twist = Twist()
        twist.linear.x = float(final_linear)
        twist.angular.z = float(final_angular)
        self.cmd_vel_pub.publish(twist)

        # Prepare response
        response.state = self.get_state(request.previous_action[LINEAR], request.previous_action[ANGULAR])
        response.reward = rw.get_reward(self.succeed, final_linear, final_angular, self.goal_distance,
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
            self.get_logger().info(
                f"Rtot: {response.reward:<8.2f}GD: {self.goal_distance:<8.2f}GA: {math.degrees(self.goal_angle):.1f}° "
                f"MinD: {self.obstacle_distance:<8.2f}m Alin: {request.action[LINEAR]:<7.2f} Aturn: {request.action[ANGULAR]:<7.2f} -> final_lin: {final_linear:.3f}"
            )
        return response



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
