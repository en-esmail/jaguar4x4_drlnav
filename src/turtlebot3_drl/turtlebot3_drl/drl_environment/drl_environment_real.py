#!/usr/bin/env python3
# Copyright 2019 ROBOTIS CO., LTD.
# ... (header omitted for brevity)

import math
import sys
import copy
import collections
import numpy as np

import time # Add this at top if missing


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

        self.local_step = 0

        # LiDAR / scan state
        self.scan_ranges = [1.0] * REAL_N_SCAN_SAMPLES
        self.obstacle_distance_front = REAL_LIDAR_DISTANCE_CAP
        self.obstacle_distance_rear = REAL_LIDAR_DISTANCE_CAP
        self.obstacle_distance_any = REAL_LIDAR_DISTANCE_CAP
        self.obstacle_distance = REAL_LIDAR_DISTANCE_CAP

        # executed actions (what robot actually did) - important to put in state
        self.last_executed_linear = 0.0
        self.last_executed_angular = 0.0

        # smoothing
        self.prev_distance = None
        self.min_history = collections.deque(maxlen=3)

        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.velo_topic, qos)

        self.goal_pose_sub = self.create_subscription(Pose, self.goal_topic, self.goal_pose_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, qos)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile=qos_profile_sensor_data)
        self.step_comm_server = self.create_service(DrlStep, 'step_comm', self.step_comm_callback)
        self.goal_comm_server = self.create_service(Goal, 'goal_comm', self.goal_comm_callback)

    def _downsample_by_min(self, arr: np.ndarray, out_size: int, max_val: float):
        if arr.size == out_size:
            return arr.copy()
        if arr.size > out_size:
            bins = np.array_split(arr, out_size)
            return np.array([np.min(b) if b.size > 0 else max_val for b in bins], dtype=np.float32)
        x_old = np.linspace(0, 1, arr.size)
        x_new = np.linspace(0, 1, out_size)
        return np.interp(x_new, x_old, arr).astype(np.float32)

    def _sliding_median(self, arr: np.ndarray, k=3):
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
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        _, _, self.robot_heading = util.euler_from_quaternion(msg.pose.pose.orientation)
        self.robot_tilt = msg.pose.pose.orientation.y

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
        """
        Robust scan processing:
          - replace NaN/inf with large cap
          - apply correction and cap
          - compute front/rear sector mins using angles (safe)
          - downsample & median-filter for NN state
        """
        ranges = np.array(msg.ranges, dtype=np.float32)

        # # sensor limits fallback
        # rng_max = msg.range_max if (hasattr(msg, 'range_max') and msg.range_max and msg.range_max > 0) else REAL_LIDAR_DISTANCE_CAP
        # rng_min = msg.range_min if (hasattr(msg, 'range_min') and msg.range_min and msg.range_min > 0) else 0.05

        # # replace nan/inf with rng_max (treat as no return / clear)
        # ranges = np.nan_to_num(ranges, nan=rng_max, posinf=rng_max, neginf=rng_max)

        # # replace values below min with rng_max (invalid/too-close returns)
        # ranges[ranges < rng_min] = rng_max

        # sensor limits fallback
        rng_max = msg.range_max if (hasattr(msg, 'range_max') and msg.range_max and msg.range_max > 0) else REAL_LIDAR_DISTANCE_CAP
        
        # --- MODIFICATION START ---
        # Force a strict minimum range. LDS-01 blind zone is ~0.12m.
        # We set 0.15m to safely filter out the noise (0.01m, 0.09m) and Error Code (0.0m).
        rng_min = 0.15
        # --- MODIFICATION END ---

        # replace nan/inf with rng_max (treat as no return / clear)
        ranges = np.nan_to_num(ranges, nan=rng_max, posinf=rng_max, neginf=rng_max)

        # replace values below min (including 0.0 errors) with rng_max (treat as clear space)
        ranges[ranges < rng_min] = rng_max


        # apply systematic correction and cap
        corrected = np.clip(ranges - REAL_LIDAR_CORRECTION, 0.0, REAL_LIDAR_DISTANCE_CAP)

        # global min (any direction)
        global_min = float(corrected.min()) if corrected.size > 0 else float(rng_max)

        # Use angle info if available for robust sector selection
        front_deg = 25.0  # +/- degrees for front
        rear_deg = 25.0   # +/- degrees for rear

        if hasattr(msg, 'angle_min') and hasattr(msg, 'angle_increment'):
            angles = (msg.angle_min + np.arange(corrected.size) * msg.angle_increment).astype(np.float32)
            ang_norm = (angles + np.pi) % (2 * np.pi) - np.pi  # normalize to [-pi, pi]

            front_mask = np.abs(ang_norm) <= np.deg2rad(front_deg)
            if np.any(front_mask):
                front_min = float(corrected[front_mask].min())
            else:
                front_min = global_min

            rear_mask = np.abs(np.abs(ang_norm) - math.pi) <= np.deg2rad(rear_deg)
            if np.any(rear_mask):
                rear_min = float(corrected[rear_mask].min())
            else:
                rear_min = global_min
        else:
            # fallback: split array into two halves
            n = corrected.size
            if n == 0:
                front_min = rear_min = global_min
            else:
                half = n // 2
                # front around indices [0..small] and [n-small..n]
                small = max(1, n // 16)
                front_candidates = np.concatenate([corrected[:small], corrected[-small:]])
                front_min = float(front_candidates.min()) if front_candidates.size > 0 else global_min
                rear_candidates = corrected[half - small: half + small] if (half - small >= 0) else corrected[half:half + small]
                rear_min = float(rear_candidates.min()) if rear_candidates.size > 0 else global_min

        # Downsample to expected size & preserve obstacles
        down = self._downsample_by_min(corrected, REAL_N_SCAN_SAMPLES, rng_max)
        down = self._sliding_median(down, k=3)
        norm = np.clip(down / REAL_LIDAR_DISTANCE_CAP, 0.0, 1.0)
        self.scan_ranges = list(norm.astype(np.float32))

        # store obstacle distances
        self.obstacle_distance_any = global_min
        self.obstacle_distance_front = front_min
        self.obstacle_distance_rear = rear_min
        # default obstacle_distance used elsewhere — mirror front (legacy)
        self.obstacle_distance = self.obstacle_distance_front

        # smoothing history
        self.min_history.append(self.obstacle_distance)
        filtered = float(np.median(self.min_history))
        if self.prev_distance is not None and abs(filtered - self.prev_distance) > 0.6:
            filtered = self.prev_distance
        self.prev_distance = filtered
        self.obstacle_distance = filtered

        # Debug
        if self.local_step % 10 == 0:
            self.get_logger().info(
                f"[scan] FrontObs={self.obstacle_distance_front:.2f}m RearObs={self.obstacle_distance_rear:.2f}m Any={self.obstacle_distance_any:.2f}m"
            )

            
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



    # def stop_reset_robot(self, success):
    #     self.cmd_vel_pub.publish(Twist())  # stop robot
    #     self.done = True


    def stop_reset_robot(self, success):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        
        # Publish stop multiple times to ensure the real robot driver receives it
        # and overrides the previous motion command
        for _ in range(10):
            self.cmd_vel_pub.publish(stop_cmd)
            time.sleep(0.01) # Small delay to allow propagation
            
        self.get_logger().info("Force Stop Command Sent x10")
        self.done = True

    def get_state(self, action_linear_previous=None, action_angular_previous=None):
        """
        Return state list of floats for RL:
          - scan ranges (REAL_N_SCAN_SAMPLES)
          - normalized goal distance
          - normalized goal angle
          - previous executed actions (linear, angular)
        Uses last_executed_* when arguments are None.
        """
        state = copy.deepcopy(self.scan_ranges)

        state.append(float(np.clip((self.goal_distance / MAX_GOAL_DISTANCE), 0, 1)))
        state.append(float(self.goal_angle) / math.pi)

        # prefer executed values if caller didn't provide
        lin_prev = self.last_executed_linear if action_linear_previous is None else float(action_linear_previous)
        ang_prev = self.last_executed_angular if action_angular_previous is None else float(action_angular_previous)

        state.append(float(lin_prev))
        state.append(float(ang_prev))

        self.local_step += 1

        if self.local_step <= 15:
            return [float(x) for x in state]

        # Success
        if self.goal_distance < REAL_THRESHOLD_GOAL:
            print("Outcome: Goal reached! :)")
            self.succeed = SUCCESS

        # Collision: use the (smoothed) obstacle_distance legacy var
        elif self.obstacle_distance < REAL_THRESHOLD_COLLISION:
            print("Collision! (wall) :(")
            self.succeed = COLLISION_WALL

        if self.succeed is not UNKNOWN:
            self.stop_reset_robot(self.succeed == SUCCESS)

        return [float(x) for x in state]

    def initalize_episode(self, response):
        self.initial_distance_to_goal = self.goal_distance
        rw.reward_initalize(self.initial_distance_to_goal)  # inform reward module
        response.state = self.get_state(0.0, 0.0)
        response.reward = 0.0
        response.done = False
        response.distance_traveled = 0.0
        return response

    def step_comm_callback(self, request, response):

        if len(request.action) == 0:
            return self.initalize_episode(response)

        # Unnormalize actions
        if ENABLE_BACKWARD:
            action_linear = float(request.action[LINEAR]) * REAL_SPEED_LINEAR_MAX
        else:
            action_linear = float((request.action[LINEAR] + 1.0) / 2.0 * REAL_SPEED_LINEAR_MAX)
        action_angular = float(request.action[ANGULAR]) * REAL_SPEED_ANGULAR_MAX

        # Clip to permitted ranges
        action_linear = float(np.clip(action_linear, -REAL_SPEED_LINEAR_MAX, REAL_SPEED_LINEAR_MAX))
        action_angular = float(np.clip(action_angular, -REAL_SPEED_ANGULAR_MAX, REAL_SPEED_ANGULAR_MAX))

        SAFE_FRONT_STOP = max(REAL_THRESHOLD_COLLISION + 0.05, 0.15)
        SAFE_REAR_STOP = SAFE_FRONT_STOP

        final_linear = action_linear
        final_angular = action_angular

        # # Safety: block forward if obstacle in front
    #    if action_linear > 0.0 and self.obstacle_distance_front < SAFE_FRONT_STOP:
     #       self.get_logger().warn(
      #          f"Front obstacle {self.obstacle_distance_front:.2f}m < {SAFE_FRONT_STOP:.2f}m -> stopping forward motion."
        #    )
       #     final_linear = 0.0

        # Safety: block backward only if obstacle behind
        #elif action_linear < 0.0 and self.obstacle_distance_rear < SAFE_REAR_STOP:
         #   self.get_logger().warn(
          #      f"Rear obstacle {self.obstacle_distance_rear:.2f}m < {SAFE_REAR_STOP:.2f}m -> stopping backward motion."
      #      )
           # final_linear = 0.0

        # Publish SAFE command
        twist = Twist()
        twist.linear.x = float(final_linear)
        twist.angular.z = float(final_angular)
        self.cmd_vel_pub.publish(twist)




        # Save executed actions so state reflects reality
        self.last_executed_linear = float(final_linear)
        self.last_executed_angular = float(final_angular)

        # Prepare response using executed actions for correct state history
        response.state = self.get_state(self.last_executed_linear, self.last_executed_angular)

        # choose min_obstacle_distance to pass to reward (use any-direction min)
        min_obs = float(min(self.obstacle_distance_any, self.obstacle_distance_front, self.obstacle_distance_rear))

        response.reward = rw.get_reward(self.succeed, final_linear, final_angular,
                                        self.goal_distance, self.goal_angle, min_obs)
        response.done = self.done
        response.success = self.succeed
        response.distance_traveled = 0.0

        if self.done:
            self.new_goal = False
            response.distance_traveled = self.total_distance
            self.succeed = UNKNOWN
            self.total_distance = 0.0
            self.local_step = 0
            self.done = False

        if self.local_step % 10 == 0:
            self.get_logger().info(
                f"Reward={response.reward:.2f} | GD={self.goal_distance:.2f}m | "
                f"GA={math.degrees(self.goal_angle):.1f}° | FrontObs={self.obstacle_distance_front:.2f}m | "
                f"RearObs={self.obstacle_distance_rear:.2f}m | Action=({request.action[LINEAR]:.2f}, {request.action[ANGULAR]:.2f}) -> "
                f"Executed=({self.last_executed_linear:.2f}, {self.last_executed_angular:.2f})"
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
    drl_environment.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
