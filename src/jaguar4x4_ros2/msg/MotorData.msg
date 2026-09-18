# motor sensor data message from Dr Robot 
std_msgs/Header header    # timestamp
int64 motor_power         # motor driver power 
int64 encoder_pos         # encoder position 
int64 encoder_vel         # encoder velocity 
int64 encoder_diff        # encoder position difference
float32 motor_temp        # motor temperature 
float32 motor_current     # motor current
