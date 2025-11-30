#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def csp_motor2_only():
    rospy.init_node("csp_motor2_only_test", anonymous=False)

    pub = rospy.Publisher(
        "/maxon/canopen_motor/joint_trajectory_controller/command",
        JointTrajectory,
        queue_size=1
    )

    rate = rospy.Rate(50)  # 50Hz (20ms)

    joint_names = ["base_link1_joint", "link1_link2_joint"]

    rospy.loginfo("=== CSP Test: Motor2 Only ===")
    rospy.loginfo("Joint1 will stay still (0 rad)")
    rospy.loginfo("Joint2 will move between 0.0 → 1.0 → -1.0 → back")

    targets = [0.0, 1.0, -1.0, 0.0]  # rad
    idx = 0

    while not rospy.is_shutdown():

        # Joint1 stays fixed at 0
        pos1 = 0.0

        # Joint2 follows step sequence
        pos2 = targets[idx]

        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = [pos1, pos2]
        point.time_from_start = rospy.Duration(0.5)  # 0.5 sec move

        traj.points.append(point)
        pub.publish(traj)

        rospy.loginfo("Motor2 Target → %.2f rad" % pos2)

        idx = (idx + 1) % len(targets)
        rospy.sleep(2.0)  # stay at target for 2 seconds

        rate.sleep()


if __name__ == "__main__":
    try:
        csp_motor2_only()
    except rospy.ROSInterruptException:
        pass

