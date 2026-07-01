std_msgs/Header header

# Base state — Pioneer odometry in world/odom frame
nav_msgs/Odometry base_odom

# EE state — T265 pose in world/odom frame
geometry_msgs/PoseStamped ee_pose

# Arm joint state estimate [J1..J5], radians — from IK(T265, base_odom)
float64[] q_arm
float64[] dq_arm

# IK quality
bool     ik_converged
float64  ik_residual_pos     # metres
float64  ik_residual_orient  # radians

# Sensor health
bool t265_valid
bool base_odom_valid
