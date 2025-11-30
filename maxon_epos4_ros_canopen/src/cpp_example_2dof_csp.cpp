#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

using namespace std;

/* global variables */
sensor_msgs::JointState joint_state;
bool js_arrived = false;

/* joint state callback */
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state = *msg;
    js_arrived = true;
}

/* print menu */
void display_menu()
{
    ROS_INFO("---------- MENU (2-DOF CSP) ----------");
    ROS_INFO("1 - driver init");
    ROS_INFO("2 - driver halt");
    ROS_INFO("3 - driver recover");
    ROS_INFO("4 - driver shutdown");
    ROS_INFO("5 - send target position to both joints");
    ROS_INFO("6 - send individual targets (joint1, joint2)");
    ROS_INFO("7 - read current joint position");
    ROS_INFO("8 - exit");
    ROS_INFO("--------------------------------------");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cpp_example_2dof_csp");
    ros::NodeHandle nh("~");

    int menu = 0;
    double pos1 = 0.0, pos2 = 0.0;

    /* service clients */
    ros::ServiceClient clt_init = nh.serviceClient<std_srvs::Trigger>("/maxon/driver/init");
    ros::ServiceClient clt_halt = nh.serviceClient<std_srvs::Trigger>("/maxon/driver/halt");
    ros::ServiceClient clt_recover = nh.serviceClient<std_srvs::Trigger>("/maxon/driver/recover");
    ros::ServiceClient clt_shutdown = nh.serviceClient<std_srvs::Trigger>("/maxon/driver/shutdown");

    /* publisher */
    ros::Publisher pub_traj = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/maxon/canopen_motor/joint_trajectory_controller/command", 
        1
    );

    /* subscriber */
    ros::Subscriber sub_js = nh.subscribe("/maxon/joint_states", 1, jointStateCallback);

    ROS_INFO("=== 2-DOF CSP Control C++ Example Started ===");
    display_menu();

    std_srvs::Trigger srv;

    while (ros::ok())
    {
        cout << "Enter choice: ";
        cin >> menu;

        switch(menu)
        {
            case 1:
                clt_init.call(srv);
                break;

            case 2:
                clt_halt.call(srv);
                break;

            case 3:
                clt_recover.call(srv);
                break;

            case 4:
                clt_shutdown.call(srv);
                break;

            case 5:
                ROS_INFO("Enter target pos (rad) for both joints:");
                cin >> pos1;
                cin >> pos2;
                {
                    trajectory_msgs::JointTrajectory traj;
                    traj.joint_names = {"base_link1_joint", "link1_link2_joint"};

                    trajectory_msgs::JointTrajectoryPoint p;
                    p.positions = {pos1, pos2};
                    p.time_from_start = ros::Duration(0.01);

                    traj.points.push_back(p);
                    pub_traj.publish(traj);

                    ROS_INFO("Sent pos1=%f, pos2=%f", pos1, pos2);
                }
                break;

            case 6:
                ROS_INFO("Enter joint1 target:");
                cin >> pos1;
                ROS_INFO("Enter joint2 target:");
                cin >> pos2;
                {
                    trajectory_msgs::JointTrajectory traj;
                    traj.joint_names = {"base_link1_joint", "link1_link2_joint"};

                    trajectory_msgs::JointTrajectoryPoint p;
                    p.positions = {pos1, pos2};
                    p.time_from_start = ros::Duration(0.01);

                    traj.points.push_back(p);
                    pub_traj.publish(traj);

                    ROS_INFO("Sent targets individually.");
                }
                break;

            case 7:
                ros::spinOnce();
                if (js_arrived)
                {
                    ROS_INFO("Joint1 = %f rad  |  Joint2 = %f rad",
                        joint_state.position[0],
                        joint_state.position[1]);
                    js_arrived = false;
                }
                else
                {
                    ROS_WARN("No JointState yet.");
                }
                break;

            case 8:
                ROS_INFO("Exiting...");
                return 0;

            default:
                ROS_WARN("Invalid menu.");
                break;
        }
    }

    return 0;
}

