#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def two_motor_csp_sync():
    rospy.init_node("two_motor_csp_sync", anonymous=False)

    pub = rospy.Publisher(
        "/maxon/canopen_motor/joint_trajectory_controller/command",
        JointTrajectory,
        queue_size=1
    )

    rospy.loginfo("=== 2-DOF CSP Sync Control Started ===")

    # Joint names must match controller config
    JOINTS = ["base_link1_joint", "link1_link2_joint"]

    # Predefined synchronized targets for both joints
    targets = [
        (0.0, 0.0),    # both go to 0 rad
        (1.0, 1.0),    # both go to +1 rad
        (-1.0, -1.0),  # both go to -1 rad
        (0.0, 0.0)     # back to 0 rad
    ]

    idx = 0

    rate = rospy.Rate(1)  # 1Hz, because we send new command every 2 sec

    while not rospy.is_shutdown():
        # Pick next sync target
        pos1, pos2 = targets[idx]

        traj = JointTrajectory()
        traj.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = [pos1, pos2]
        point.time_from_start = rospy.Duration(2.0)  # BOTH axes reach in exactly 2 seconds

        traj.points.append(point)
        pub.publish(traj)

        rospy.loginfo("➡ New Target: Joint1 = %.2f rad, Joint2 = %.2f rad" % (pos1, pos2))

        idx = (idx + 1) % len(targets)

        rospy.sleep(2.0)  # maintain this target for 2 seconds


if __name__ == "__main__":
    try:
        two_motor_csp_sync()
    except rospy.ROSInterruptException:
        pass

