# hole_in_the_wall (WALL-Z)

## 1. Introduction

This repository contains simulation models for a robotic arm that can play the game [Hole in the Wall](https://en.wikipedia.org/wiki/Hole_in_the_Wall_(American_game_show)) that I have decided to call **WALL-Z**. Currently a work in progress with the Alpha version out.

The eventual goal is to create a system with the following characteristics:
- Able to take an image input with a clear 'hole' and determine if the robot arm has a configuration that would allow it to pass through
- Create an algorithm that can intake the joint and link parameters (link length, joint type, joint amount etc.) to create a modular solution for any reasonable DoF arm (most likely for 1 DoF up to 7 DoF)
- Control the arm to move into the calculated position in both RViz and Gazebo using a combination of ros2_control and moveit2
- Control an IRL arm and camera with the same functionality

The development and test environment is as follows:
- Ubuntu 20.04 + ROS Foxy

## 2. Update History
- (2023-08-26): Updated README.md
- (2023-08-30): Functional Gazebo model
- (2023-09-02): Functional image processing script for 2DOF robot
- (2023-09-05): Functional service / client node for solving and publishing joint commands
- (2023-09-06): Alpha version complete! [Watch video demonstration here](https://youtu.be/OODFsPO2H0o).

## 3. Installation
- ### 3.1 Install [ROS2 Foxy](https://docs.ros.org/en/ros2_documentation/foxy/Installation.html) 

- ### 3.2 Install [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)  

- ### 3.3 Install [ros2_control](https://control.ros.org/master/index.html)  
    ```bash
    $ sudo apt install ros-foxy-ros2-control
    $ sudo apt install ros-foxy-ros2-controllers
    ```

- ### 3.4 Create a workspace
    ```bash
    # Skip this step if you already have a target workspace
    $ cd ~
    $ mkdir -p dev_ws/src
    ```

- ### 3.5 Obtain source code of "hole_in_the_wall" repository
    ```bash
    $ cd ~/dev_ws/src
    $ git clone https://github.com/RaZeragon/hole_in_the_wall.git
    ```

- ### 3.6 Update "hole_in_the_wall" repository 
    ```bash
    $ cd ~/dev_ws/src/hole_in_the_wall
    $ git pull
    ```

- ### 3.7 Install dependencies
    ```bash
    # Remember to source ros2 environment settings first
    $ source /opt/ros/foxy/setup.bash
    $ cd ~/dev_ws/src/
    $ rosdep update
    $ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
    ```

- ### 3.8 Build hole_in_the_wall
    ```bash
    # Remember to source ros2 environment settings first
    $ source /opt/ros/foxy/setup.bash
    $ cd ~/dev_ws/
    # build all packages
    $ colcon build
    
    # build selected packages
    $ colcon build --packages-select hitw_description
    ```

- ### 3.9 Create Wall model in .gazebo/models
    ```bash
    $ cd ~/dev_ws/src/hole_in_the_wall/hitw_gazebo/models
    $ cp /Wall ~/.gazebo/models/Wall
    # Make sure your gazebo plugin path is set for ~/.gazebo/models
    ```

## 4. Package Introduction
- ### 4.1 hitw_description
    This package contains robot description files and 3D models of the HITW robot. Models can be displayed in RViz by the following launch file:
    ```bash
    $ cd ~/dev_ws/
    
    # Launch a view-only model of the arm
    $ ros2 launch hitw_description view_arm.launch.py

    # Launch the arm with joint-state GUI
    $ ros2 launch hitw_description arm.launch.py
    ```

- ### 4.2 hitw_controllers 
    This package contains config files for ros2_control.

- ### 4.3 hitw_hardware
    This package contains the plugin for controlling the arm.

- ### 4.4 hitw_algorithm
    This package contains the code for analyzing the hole and determining if the current arm can fit inside. 
    ```bash
    $ cd ~/dev_ws/
    
    # Note: The following two nodes need to be run in separate terminals to function properly
    # Run the service node for calculating the robot pose
    $ ros2 run hitw_algorithm robotpose_service

    # LRun the client node for calculating the robot pose
    $ ros2 run hitw_algorithm robotpose_service
    ```

- ### 4.5 hitw_gazebo
    This package contains the Gazebo launch files.
    ```bash
    $ cd ~/dev_ws/
    $ ros2 launch hitw_gazebo arm_gazebo.launch.py
    ```
- ### 4.6 hitw_msgs
    This package contains custom services and msgs for the hole in the wall robot.

## 5. Usage
- ### 5.1 Setup
    Open a new terminal and use the following commands. This will start the Gazebo simulation of the arm.
    ```bash
    $ cd ~/dev_ws/
    $ source /opt/ros/foxy/setup.bash
    $ . install/setup.bash
    $ ros2 launch hitw_gazebo arm_gazebo.launch.py
    ```

    Open up a second terminal and use the following commands. This will start the service node for calculating the robot's pose based on the camera's image.
    ```bash
    $ cd ~/dev_ws/
    $ source /opt/ros/foxy/setup.bash
    $ . install/setup.bash
    $ ros2 run hitw_algorithm robotpose_service
    ```

    Open up a third terminal and use the following commands. This will start the client node for calculating the robot's pose based on the camera's image.
    ```bash
    $ cd ~/dev_ws/
    $ source /opt/ros/foxy/setup.bash
    $ . install/setup.bash
    $ ros2 run hitw_algorithm robotpose_client.
    ```

    After a few seconds, an image showing the camera feed should show up. Hit spacebar to move to the next image which will showcase the edge-detected and thresholded image. Hit spacebar again to move to the final image which will showcase the original image with the calculated robot pose shown. Hit spacebar again and the robot should now move into the indicated position. Example video can be found [here](https://youtu.be/OODFsPO2H0o).

## 6. To Do
- Increase DoF (up to 7)
- Update image_processing script (check the file for more detailed notes)
- Add function to allow user to input an image as a launch argument
- Combine and/or refine the robotpose service/client
- Fix the Gazebo jittering and allow for smooth movement

## 7. References
- [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos/tree/foxy)
- [ROS Control](https://www.rosroboticslearning.com/ros-control)
- [Making a Mobile Robot #12 - ros2_control Concept & Simulation](https://articulatedrobotics.xyz/mobile-robot-12-ros2-control/)
- [Bresenham's Circle Drawing Algorithm](https://www.geeksforgeeks.org/bresenhams-circle-drawing-algorithm/)
