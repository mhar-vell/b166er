#!/usr/bin/env python3
"""Mata joint_state_publisher_gui de sessões anteriores via shutdown ROS."""
import rospy
import os

rospy.init_node('kill_gui_zombie', anonymous=True, disable_rostime=True)
try:
    rospy.wait_for_service('/joint_state_publisher_gui/get_loggers', timeout=1.0)
    import subprocess
    subprocess.run(['rosnode', 'kill', '/joint_state_publisher_gui'], check=False)
    rospy.loginfo('[kill_gui_zombie] joint_state_publisher_gui terminado')
except Exception:
    rospy.loginfo('[kill_gui_zombie] joint_state_publisher_gui nao encontrado (ok)')
