# hole_in_the_wall

## 1. Introduction

&ensp;&ensp;&ensp;&ensp;This repository contains simulation models for a robotic arm that can play the game [Hole in the Wall](https://en.wikipedia.org/wiki/Hole_in_the_Wall_(American_game_show)). Currently a work in progress.

&ensp;&ensp;&ensp;&ensp;The eventual goal is to create a system with the following characteristics:
- Able to take an image input with a clear 'hole' and determine if the robot arm has a configuration that would allow it to pass through
- Create an algorithm that can intake the joint and link parameters (link length, joint type, joint amount etc.) to create a modular solution for any reasonable DoF arm (most likely for 1 DoF up to 7 DoF)
- Control the arm to move into the calculated position in both RViz and Gazebo using a combination of ros2_control and moveit2
- Control an IRL arm and camera with the same functionality

&ensp;&ensp;&ensp;&ensp;The development and test environment is as follows:
- Ubuntu 20.04 + ROS Foxy

## 2. Update History
- (2023-08-26)
    - Updated README.md

## 3. Installation
- ### 3.1 Install [ROS2 Foxy](https://docs.ros.org/en/ros2_documentation/foxy/Installation.html) 

- ### 3.2 Install [ros2_control](https://control.ros.org/master/index.html)  
    ```bash
    $ sudo apt install ros-foxy-ros2-control
    $ sudo apt install ros-foxy-ros2-controllers
    ```

- ### 3.3 Create a workspace
    ```bash
    # Skip this step if you already have a target workspace
    $ cd ~
    $ mkdir -p dev_ws/src
    ```

- ### 3.4 Obtain source code of "hole_in_the_wall" repository
    ```bash
    $ cd ~/dev_ws/src
    $ git clone https://github.com/RaZeragon/hole_in_the_wall.git
    ```

- ### 3.5 Update "hole_in_the_wall" repository 
    ```bash
    $ cd ~/dev_ws/src/hole_in_the_wall
    $ git pull
    ```

- ### 3.6 Install dependencies
    ```bash
    # Remember to source ros2 environment settings first
    $ source /opt/ros/foxy/setup.bash
    $ cd ~/dev_ws/src/
    $ rosdep update
    $ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```

- ### 3.7 Build hole_in_the_wall
    ```bash
    # Remember to source ros2 environment settings first
    $ source /opt/ros/foxy/setup.bash
    $ cd ~/dev_ws/
    # build all packages
    $ colcon build
    
    # build selected packages
    $ colcon build --packages-select hitw_description
    ```

## 4. Package Introduction
- ### 4.1 hitw_description
    This package contains robot description files and 3D models of the HITW robot. Models can be displayed in RViz by the following launch file:
    ```bash
    $ cd ~/dev_ws/
    $ ros2 launch hitw_description view_arm.launch.py
    ```

- ### 4.2 hitw_controllers 
    This package contains config files for ros2_control.

- ### 4.3 hitw_hardware
    This package contains the plugin for controlling the arm.

## 5. References
- [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos/tree/foxy)
- [ROS Control](https://www.rosroboticslearning.com/ros-control)
- [Making a Mobile Robot #12 - ros2_control Concept & Simulation](https://articulatedrobotics.xyz/mobile-robot-12-ros2-control/)