#!/usr/bin/env python3
# filepath: ~/b166er/src/b166er_robot/scripts/test_gripper.py

import rospy
from movemaster_msg.msg import setpoint

def test_gripper():
    rospy.init_node('test_gripper')
    pub = rospy.Publisher('/setpoint', setpoint, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Create message with SetGRIP True
    msg = setpoint()
    msg.Set1 = 0
    msg.Set2 = 0
    msg.Set3 = 0
    msg.Set4 = 0
    msg.Set5 = 0
    msg.SetGRIP = True
    msg.EmergencyStop = False
    msg.GoHome = 0

    rospy.loginfo("Sending SetGRIP=True to gripper...")
    pub.publish(msg)
    rospy.sleep(2)

    # Create message with SetGRIP False
    msg.SetGRIP = False
    rospy.loginfo("Sending SetGRIP=False to gripper...")
    pub.publish(msg)
    rospy.sleep(2)

if __name__ == '__main__':
    try:
        test_gripper()
    except rospy.ROSInterruptException:
        pass