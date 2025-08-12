#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class IMUMonitor:
    def __init__(self):
        rospy.init_node('imu_monitor')
        
        # Subscribe to IMU data
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        
        rospy.loginfo("IMU Monitor started. Monitoring orientation and stability...")

    def imu_callback(self, msg):
        # Extract orientation
        quaternion = [msg.orientation.x, msg.orientation.y, 
                     msg.orientation.z, msg.orientation.w]
        
        # Convert to Euler angles
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        # Convert to degrees for readability
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # Calculate tilt magnitude
        tilt_magnitude = math.sqrt(roll**2 + pitch**2)
        tilt_deg = math.degrees(tilt_magnitude)
        
        # Print status every second (approximately)
        if hasattr(self, 'last_print_time'):
            if rospy.Time.now() - self.last_print_time < rospy.Duration(1.0):
                return
        
        self.last_print_time = rospy.Time.now()
        
        print(f"\n=== IMU Status ===")
        print(f"Roll:  {roll_deg:6.2f}°")
        print(f"Pitch: {pitch_deg:6.2f}°")
        print(f"Yaw:   {yaw_deg:6.2f}°")
        print(f"Tilt:  {tilt_deg:6.2f}°")
        
        # Stability warning
        if tilt_deg > 15:
            print(f"⚠️  WARNING: High tilt angle!")
        elif tilt_deg > 30:
            print(f"🚨 DANGER: Excessive tilt!")
        else:
            print(f"✅ Stable")
        
        # Angular velocity
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z
        print(f"Angular velocity: x={wx:.3f}, y={wy:.3f}, z={wz:.3f} rad/s")
        
        # Linear acceleration
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        print(f"Acceleration: x={ax:.3f}, y={ay:.3f}, z={az:.3f} m/s²")

if __name__ == '__main__':
    try:
        monitor = IMUMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass