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


def keyboard_csp_sync_two():
    rospy.init_node("keyboard_csp_control_sync2", anonymous=False)

    pub = rospy.Publisher(
        "/maxon/canopen_motor/joint_trajectory_controller/command",
        JointTrajectory,
        queue_size=1
    )

    joint_names = ["base_link1_joint", "link1_link2_joint"]

    # two joint positions
    pos1 = 0.0
    pos2 = 0.0

    step = 0.1  # rad

    rospy.loginfo("=== CSP Keyboard Control (Two joints synchronized) ===")
    rospy.loginfo("  a : both +0.1 rad")
    rospy.loginfo("  s : both -0.1 rad")
    rospy.loginfo("  q : quit")
    rospy.loginfo("=====================================================")

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        key = get_key()

        if key == 'a':
            pos1 += step
            pos2 += step
            rospy.loginfo("Both joints +0.1 → J1=%.2f, J2=%.2f" % (pos1, pos2))

        elif key == 's':
            pos1 -= step
            pos2 -= step
            rospy.loginfo("Both joints -0.1 → J1=%.2f, J2=%.2f" % (pos1, pos2))

        elif key == 'q':
            rospy.loginfo("Quit CSP keyboard control.")
            break

        # Create CSP trajectory message
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [pos1, pos2]
        point.time_from_start = rospy.Duration(0.1)

        traj.points.append(point)
        pub.publish(traj)

        rate.sleep()


if __name__ == "__main__":
    try:
        keyboard_csp_sync_two()
    except rospy.ROSInterruptException:
        pass

