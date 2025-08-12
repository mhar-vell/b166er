#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from tf.transformations import euler_from_quaternion

class StabilityController:
    def __init__(self):
        rospy.init_node('stability_controller')
        
        # Parameters
        self.max_tilt_angle = rospy.get_param('~max_tilt_angle', 0.3)  # 17 degrees
        self.stability_factor = 1.0
        
        # Publishers
        self.cmd_vel_filtered_pub = rospy.Publisher('/cmd_vel_filtered', Twist, queue_size=1)
        self.stability_pub = rospy.Publisher('/stability_metric', Float64, queue_size=1)
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=1)
        
        # Subscribers
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel_raw', Twist, self.cmd_vel_callback)
        
        rospy.loginfo("Stability Controller initialized")

    def imu_callback(self, msg):
        # Extract orientation
        quaternion = [msg.orientation.x, msg.orientation.y, 
                     msg.orientation.z, msg.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        # Calculate tilt magnitude
        tilt_magnitude = math.sqrt(roll**2 + pitch**2)
        
        # Update stability factor (0 = unstable, 1 = stable)
        self.stability_factor = max(0.0, 1.0 - (tilt_magnitude / self.max_tilt_angle))
        
        # Publish stability metric
        stability_msg = Float64()
        stability_msg.data = self.stability_factor
        self.stability_pub.publish(stability_msg)
        
        # Emergency stop if too unstable
        if self.stability_factor < 0.3:
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)
            rospy.logwarn(f"EMERGENCY STOP: Low stability {self.stability_factor:.2f}")

    def cmd_vel_callback(self, msg):
        # Filter velocity commands based on stability
        filtered_cmd = Twist()
        
        # Scale velocity by stability factor
        filtered_cmd.linear.x = msg.linear.x * self.stability_factor
        filtered_cmd.linear.y = msg.linear.y * self.stability_factor
        filtered_cmd.angular.z = msg.angular.z * self.stability_factor
        
        self.cmd_vel_filtered_pub.publish(filtered_cmd)

if __name__ == '__main__':
    try:
        controller = StabilityController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass