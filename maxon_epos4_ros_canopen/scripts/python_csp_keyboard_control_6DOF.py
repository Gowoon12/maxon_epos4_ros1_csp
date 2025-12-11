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


def keyboard_csp_sync_six():
    rospy.init_node("keyboard_csp_control_sync6", anonymous=False)

    pub = rospy.Publisher(
        "/maxon/canopen_motor/joint_trajectory_controller/command",
        JointTrajectory,
        queue_size=1
    )

    # joint names - MUST match your YAML
    joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    # initial positions
    pos = [0.0] * 6

    # different step sizes:
    # joints 1~4 = 0.1 rad
    # joints 5~6 = 0.01 rad
    step = [0.1, 0.1, 0.1, 0.1, 0.03, 0.03]

    rospy.loginfo("============ CSP Keyboard Control (6 DOF sync) ============")
    rospy.loginfo("  a : all joints +step (1~4:0.1, 5~6:0.01 rad)")
    rospy.loginfo("  s : all joints -step (1~4:0.1, 5~6:0.01 rad)")
    rospy.loginfo("  q : quit")
    rospy.loginfo("===========================================================")

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        key = get_key()

        if key == 'a':
            pos = [pos[i] + step[i] for i in range(6)]
            rospy.loginfo("[+] New positions: " + ", ".join("%.3f" % p for p in pos))

        elif key == 's':
            pos = [pos[i] - step[i] for i in range(6)]
            rospy.loginfo("[-] New positions: " + ", ".join("%.3f" % p for p in pos))

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
        keyboard_csp_sync_six()
    except rospy.ROSInterruptException:
        pass

