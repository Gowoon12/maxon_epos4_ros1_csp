# maxon_epos4_ros1_csp

+ This project was developed at the Medical Robotics and Intelligent Control Laboratory, Department of Mechanical Engineering, Chonnam National University, as part of preliminary testing for real-time multi-axis robot control.

+ This project provides a guide for controlling Maxon motors using the EPOS4 controller in Cyclic Synchronous Position (CSP) mode on a ROS-based system.

+ The code is primarily based on the [this repository](https://github.com/Roboprotos/maxon_epos4_ros1) and includes several custom modifications. An example for controlling two motors simultaneously is also implemented.

+ This package has been tested on Ubuntu 20.04 LTS with ROS Noetic.

+ This README provides only a summary of the key information.

+ For detailed explanations and setup instructions, please refer to the shared [PPT](https://drive.google.com/file/d/1SvXIRfTnIf9pXATsIN48US0iBAz6lbBT/view?usp=sharing)


## Hardware setup 

+ For information on the CAN–USB cable, CAN wiring, and DIP switch configuration, please refer to the [PPT](https://drive.google.com/file/d/1SvXIRfTnIf9pXATsIN48US0iBAz6lbBT/view?usp=sharing).

+ You must configure CANopen settings in EPOS Studio. Additionally, set the Interpolation Time Period and CAN bit rate in the Object Dictionary. After completing the configuration, export the DCF file and copy it into the config directory.

+ The DCF file must be modified according to the attached presentation, specifically for the following Object Dictionary entries: [2200], [2200sub0], [2200sub1], [2200sub2], [2200sub3], and [6098].


## Catkin workspace

+ To avoid dependency conflicts with an existing catkin_ws, it is recommended to create a new catkin workspace:

      mkdir -p ~/maxon_ws/src
      cd ~/maxon_ws
      catkin_make
      source ~/maxon_ws/devel/setup.bash


## Installing the maxon_epos4_ros1 Package

+ When building the package, a full catkin_make may cause errors. You should build only the maxon_epos4_ros_canopen package.

      cd ~/maxon_ws/src
      git clone https://github.com/Roboprotos/maxon_epos4_ros1.git
      cd ..
      catkin_make --pkg maxon_epos4_ros_canopen
      source ~/maxon_ws/devel/setup.bash



## CANopen Setup

+ After connecting the CAN–USB interface, verify the connection using:

      lsusb

+ Configure the CAN interface and bring up the CAN interface 

      sudo ip link set can0 type can bitrate 1000000
      sudo ip link set can0 up
      candump can0


## 1 DOF PPM control 

+ Note: In every terminal, run:

      source ~/maxon_ws/devel/setup.bash

+ Start ROS master:

      roscore

+ Launch the 1-DOF PPM controller:

      roslaunch maxon_epos4_ros_canopen maxon_epos4_canopen_motor_1dof_ppm.launch

+ Initialize the driver:

      rosservice call /maxon/driver/init

+ Test with a manual publish:

      rostopic pub /maxon/canopen_motor/base_link1_joint_position_controller/command std_msgs/Float64 -- 10

+ Run the Python example:

      rosrun maxon_epos4_ros_canopen python_example_1dof_ppm.py

+ Run the C++ example:

      rosrun maxon_epos4_ros_canopen cpp_example_1dof_ppm

+ Check available topics and echo joint topic:

      rostopic list
      rostopic echo /maxon/joint/
  

## 2 DOF CSP control 

+ Note: In every terminal, run:

      source ~/maxon_ws/devel/setup.bash

+ Start ROS master:

      roscore

+ Launch the 2-DOF CSP controller:

      roslaunch maxon_epos4_ros_canopen maxon_epos4_canopen_motor_2dof_csp.launch

+ Initialize the driver:

      rosservice call /maxon/driver/init

+ Run the Python example (newly created):

      rosrun maxon_epos4_ros_canopen python_example_2dof_csp.py

+ Check available topics and echo joint topic:

      rostopic list
      rostopic echo /maxon/joint/


## Code Description

+ The provided code must be customized according to the number of motors and the mechanical configuration of your actual robot system.

+ For systems with two or more degrees of freedom, additional nodes must be added and configured accordingly. 

+ Relevant configuration files are located in:

  maxon_epos4_ros_canopen/config
   ├── controller_2dof_csp.yaml
   └── node_layer_2dof_csp.yaml

  maxon_epos4_ros_canopen/scripts 
   ├── python_example_1dof_ppm.py (provided by the original referenced repository)
   ├── python_example_2dof_csp.py
   ├── python_csp_keyboard_control.py
   ├── python_example_csp_motor2_only.py
   └── python_csp_realtime_test.py
