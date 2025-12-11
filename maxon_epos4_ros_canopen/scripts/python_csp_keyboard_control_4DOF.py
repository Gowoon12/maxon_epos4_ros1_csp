#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import termios
import tty
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def get_key():
    """Reads a single key press (non-blocking style)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key


def keyboard_csp_sync_four():
    rospy.init_node("keyboard_csp_control_sync4", anonymous=False)

    pub = rospy.Publisher(
        "/maxon/canopen_motor/joint_trajectory_controller/command",
        JointTrajectory,
        queue_size=1
    )

    # 4 DOF joint names (YAML & URDF와 반드시 동일해야 한다)
    joint_names = ["joint1", "joint2", "joint3", "joint4"]

    # initial positions
    pos = [0.0, 0.0, 0.0, 0.0]

    step = 0.1  # rad

    rospy.loginfo("=== CSP Keyboard Control (FOUR joints synchronized) ===")
    rospy.loginfo("  a : all joints +0.1 rad")
    rospy.loginfo("  s : all joints -0.1 rad")
    rospy.loginfo("  q : quit")
    rospy.loginfo("========================================================")

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        key = get_key()

        if key == 'a':
            pos = [p + step for p in pos]
            rospy.loginfo("All +0.1 → J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f" % tuple(pos))

        elif key == 's':
            pos = [p - step for p in pos]
            rospy.loginfo("All -0.1 → J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f" % tuple(pos))

        elif key == 'q':
            rospy.loginfo("Quit CSP keyboard control.")
            break

        # Create CSP trajectory message
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = pos
        point.time_from_start = rospy.Duration(0.1)

        traj.points.append(point)
        pub.publish(traj)

        rate.sleep()


if __name__ == "__main__":
    try:
        keyboard_csp_sync_four()
    except rospy.ROSInterruptException:
        pass

