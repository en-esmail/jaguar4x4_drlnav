# ===================================================================== #
#                           GENERAL SETTINGS                            #
# ===================================================================== #

ENABLE_BACKWARD          = True  # Enable backward movement of the robot
ENABLE_STACKING          = False    # Enable processing multiple consecutive scan frames at every observation step
ENABLE_VISUAL            = False     # Meant to be used only during evaluation/testing phase
ENABLE_TRUE_RANDOM_GOALS = False    # If false, goals are selected semi-randomly from a list of known valid goal positions
ENABLE_DYNAMIC_GOALS     = False    # If true, goal difficulty (distance) is adapted according to current success rate
MODEL_STORE_INTERVAL     = 100      # Store the model weights every N episodes
GRAPH_DRAW_INTERVAL      = 10       # Draw the graph every N episodes (drawing too often will slow down training)
GRAPH_AVERAGE_REWARD     = 10       # Average the reward graph over every N episodes


# ===================================================================== #
#                         ENVIRONMENT SETTINGS                          #
# ===================================================================== #

# --- SIMULATION ENVIRONMENT SETTINGS ---
REWARD_FUNCTION = "B"           # Defined in reward.py
EPISODE_TIMEOUT_SECONDS = 50    # Number of seconds after which episode timeout occurs

TOPIC_SCAN = 'scan'
TOPIC_VELO = 'cmd_vel'
TOPIC_ODOM = 'odom'

# EPISODE_TIMEOUT_SECONDS     = 50    # Number of seconds after which episode timeout occurs
# ARENA_LENGTH                = 4.2   # meters
# ARENA_WIDTH                 = 4.2   # meters
# SPEED_LINEAR_MAX            = 0.22  # m/s
# SPEED_ANGULAR_MAX           = 2.0   # rad/s

# LIDAR_DISTANCE_CAP          = 3.5   # meters
# THRESHOLD_COLLISION         = 0.13      #0.13  # meters
# THREHSOLD_GOAL              = 0.20  # meters

# OBSTACLE_RADIUS             = 0.16  # meters
# MAX_NUMBER_OBSTACLES        = 6
# ENABLE_MOTOR_NOISE          = False # Add normally distributed noise to motor output to simulate hardware imperfections

EPISODE_TIMEOUT_SECONDS     = 50    # Number of seconds after which episode timeout occurs
ARENA_LENGTH                = 10   # meters
ARENA_WIDTH                 = 10   # meters
SPEED_LINEAR_MAX            = 0.10 #0.22  # m/s
SPEED_ANGULAR_MAX           = 1.0 #2.0   # rad/s

LIDAR_DISTANCE_CAP          = 2.5 # meters
THRESHOLD_COLLISION         = 0.18  #0.13  #meters
THREHSOLD_GOAL              = 0.20  # meters

OBSTACLE_RADIUS             = 0.16  # meters
MAX_NUMBER_OBSTACLES        = 6
ENABLE_MOTOR_NOISE          = False # Add normally distributed noise to motor output to simulate hardware imperfections

# --- REAL ROBOT ENVIRONMENT SETTINGS ---
REAL_TOPIC_SCAN  = '/scan'
REAL_TOPIC_VELO  = '/ros_drrobot_motor_cmd' #'/mobile_base/commands/velocity' # '/cmd_vel_mux/input/navi' #
REAL_TOPIC_ODOM  = '/odom'

# LiDAR density count your robot is providing
# NOTE: If you change this value you also have to modify
# NUM_SCAN_SAMPLES for the model in drl_environment.py
# e.g. if you increase this by 320 samples also increase
# NUM_SCAN_SAMPLES by 320 samples.
REAL_N_SCAN_SAMPLES         = 40  #was 40

REAL_ARENA_LENGTH           = 3.2 #3.2	# 2.51  # meters 4.2
REAL_ARENA_WIDTH            = 2.5 #2.5		#1.675   # meters 4.2
REAL_SPEED_LINEAR_MAX       = 0.16 # 1 0.22in m/s 0.22
REAL_SPEED_ANGULAR_MAX      = 0.60 # 2.0 in rad/s 1.8

REAL_LIDAR_CORRECTION       = 0.15 #0.06 # 0.01 0.05 meters, subtracted from the real LiDAR values was 0.40 0.02
REAL_LIDAR_DISTANCE_CAP     = 2.5 # meters, 5.6 3.5 1 scan distances are capped this value
REAL_THRESHOLD_COLLISION    = 0.19 # 0.32 0.23 meters, 0.17 0.2 0.11 minimum distance to an object that counts as a collision
REAL_THRESHOLD_GOAL         = 0.35  # 0.20 0.35 meters, minimum distance to goal that counts as reaching the goal

# ===================================================================== #
#                       DRL ALGORITHM SETTINGS                          #
# ===================================================================== #

# DRL parameters
REWARD_FUNCTION = "B"       # Defined in reward.py
ACTION_SIZE     = 2         # Not used for DQN, see DQN_ACTION_SIZE
HIDDEN_SIZE     = 512       # Number of neurons in hidden layers

BATCH_SIZE      = 128       # Number of samples per training batch
BUFFER_SIZE     = 1000000   # Number of samples stored in replay buffer before FIFO
DISCOUNT_FACTOR = 0.99
LEARNING_RATE   = 0.003
TAU             = 0.003

OBSERVE_STEPS   = 25000     # At training start random actions are taken for N steps for better exploration
STEP_TIME       = 0.01      # Delay between steps, can be set to 0
EPSILON_DECAY   = 0.9995    # Epsilon decay per step
EPSILON_MINIMUM = 0.0
# DQN parameters
DQN_ACTION_SIZE = 5
TARGET_UPDATE_FREQUENCY = 1000

# DDPG parameters

# TD3 parameters
POLICY_NOISE            = 0.2
POLICY_NOISE_CLIP       = 0.5
POLICY_UPDATE_FREQUENCY = 2

# Stacking
STACK_DEPTH = 3             # Number of subsequent frames processed per step
FRAME_SKIP  = 4             # Number of frames skipped in between subsequent frames

# Episode outcome enumeration
UNKNOWN = 0
SUCCESS = 1
COLLISION_WALL = 2
COLLISION_OBSTACLE = 3
TIMEOUT = 4
TUMBLE = 5
RESULTS_NUM = 6
