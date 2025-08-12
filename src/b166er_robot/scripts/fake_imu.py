#!/usr/bin/env python3
# fake_imu_publisher.py

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

class FakeIMUPublisher:
    def __init__(self):
        rospy.init_node('fake_imu_publisher')
        
        self.pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.rate = rospy.Rate(50)  # 50 Hz
        
        rospy.loginfo("Fake IMU Publisher started")
        
    def run(self):
        while not rospy.is_shutdown():
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"
            
            # Simulate small movements and tilts
            t = rospy.Time.now().to_sec()
            roll = 0.05 * math.sin(0.5 * t)    # Small roll oscillation
            pitch = 0.03 * math.cos(0.3 * t)   # Small pitch oscillation  
            yaw = 0.1 * t                      # Slow rotation
            
            # Convert to quaternion
            q = quaternion_from_euler(roll, pitch, yaw)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            
            # Add angular velocity
            imu_msg.angular_velocity.x = 0.02 * math.cos(t)
            imu_msg.angular_velocity.y = 0.01 * math.sin(t)
            imu_msg.angular_velocity.z = 0.1
            
            # Add linear acceleration
            imu_msg.linear_acceleration.x = 0.1 * math.sin(t)
            imu_msg.linear_acceleration.y = 0.05 * math.cos(t)
            imu_msg.linear_acceleration.z = 9.81  # Gravity
            
            # Set covariances
            imu_msg.orientation_covariance[0] = 0.01
            imu_msg.orientation_covariance[4] = 0.01
            imu_msg.orientation_covariance[8] = 0.01
            
            self.pub.publish(imu_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        fake_imu = FakeIMUPublisher()
        fake_imu.run()
    except rospy.ROSInterruptException:
        pass