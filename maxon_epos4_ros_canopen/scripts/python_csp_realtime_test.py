#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def realtime_test():
    rospy.init_node("csp_realtime_test", anonymous=False)

    pub = rospy.Publisher(
        "/maxon/canopen_motor/joint_trajectory_controller/command",
        JointTrajectory,
        queue_size=1
    )

    rate_hz = 100
    rate = rospy.Rate(rate_hz)

    joints = ["base_link1_joint", "link1_link2_joint"]

    prev_time = time.time()

    rospy.loginfo("=== CSP 100Hz Real-time Test START ===")

    while not rospy.is_shutdown():
        # Make a tiny movement (안전하게)
        traj = JointTrajectory()
        traj.joint_names = joints

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0]
        point.time_from_start = rospy.Duration(0.01)  # 100Hz trajectory timing
        traj.points.append(point)

        pub.publish(traj)

        # Measure publish interval
        now = time.time()
        dt = now - prev_time
        prev_time = now

        rospy.loginfo("Δt = %.6f sec   (%.1f Hz)" % (dt, 1.0/dt if dt > 0 else 0))

        rate.sleep()


if __name__ == "__main__":
    try:
        realtime_test()
    except rospy.ROSInterruptException:
        pass

