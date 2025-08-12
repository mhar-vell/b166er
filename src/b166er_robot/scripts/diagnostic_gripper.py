#!/usr/bin/env python3
# filepath: ~/b166er/src/b166er_robot/scripts/diagnostic_gripper.py

import rospy
from movemaster_msg.msg import setpoint

def publish_grip(pub, grip_value):
    msg = setpoint()
    
    # First, let's check what fields are actually available
    rospy.loginfo(f"Available fields in setpoint message: {dir(msg)}")
    
    # Try to set only the fields that exist
    try:
        if hasattr(msg, 'set_GRIP'):
            msg.set_GRIP = grip_value
        elif hasattr(msg, 'SetGRIP'):
            msg.SetGRIP = grip_value
        elif hasattr(msg, 'grip'):
            msg.grip = grip_value
        else:
            rospy.logerr("No gripper field found in message!")
            return
            
        # Set other fields only if they exist
        if hasattr(msg, 'emergency_stop'):
            msg.emergency_stop = False
        elif hasattr(msg, 'EmergencyStop'):
            msg.EmergencyStop = False
            
    except Exception as e:
        rospy.logerr(f"Error setting message fields: {e}")
        return
        
    pub.publish(msg)
    rospy.loginfo(f"Published gripper command: {grip_value}")

def check_topic():
    topics = rospy.get_published_topics()
    for topic, msg_type in topics:
        if '/setpoint' in topic and 'movemaster_msg/setpoint' in msg_type:
            return True
    return False

def diagnostic_gripper():
    rospy.init_node('diagnostic_gripper')
    pub = rospy.Publisher('/setpoint', setpoint, queue_size=10)
    rospy.sleep(1)

    # Step 1: Check message structure
    rospy.loginfo("Checking movemaster_msg/setpoint structure...")
    msg = setpoint()
    rospy.loginfo(f"Message fields: {[attr for attr in dir(msg) if not attr.startswith('_')]}")

    # Step 2: Check if topic exists
    if not check_topic():
        rospy.logerr("Topic /setpoint with type movemaster_msg/setpoint not found. Check if movemaster node is running.")
        return

    rospy.loginfo("Topic /setpoint found. Testing gripper software communication...")

    # Step 3: Publish SetGRIP True
    publish_grip(pub, True)
    rospy.sleep(2)

    # Step 4: Publish SetGRIP False
    publish_grip(pub, False)
    rospy.sleep(2)

    # Step 5: Diagnostic output
    rospy.loginfo("If the gripper did not move, check hardware connections and power.")
    rospy.loginfo("If the gripper moved, software communication is working.")

if __name__ == '__main__':
    try:
        diagnostic_gripper()
    except rospy.ROSInterruptException:
        pass