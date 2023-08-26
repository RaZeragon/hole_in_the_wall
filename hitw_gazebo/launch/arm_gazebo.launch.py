#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # Setting default paths
    pkg_hitw_description = get_package_share_directory('hitw_description')
    pkg_hitw_controllers = get_package_share_directory('hitw_controllers')
    pkg_hitw_gazebo = get_package_share_directory('hitw_gazebo')
    default_urdf_model_path = os.path.join(pkg_hitw_description, 'urdf/hitw_arm_macro.urdf.xacro')

    # Launch configuration variables specific to simulation
    urdf_model = LaunchConfiguration('urdf_model')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')
    
    # Declare the launch arguments  
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file'
    )
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher'
    )
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_prefix = DeclareLaunchArgument(
            name="prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers' configuration \
            have to be updated.",
    )

    # Start World
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_hitw_gazebo, 'launch', 'start_world.launch.py')
        )
    )

    # Get URDF via xacro
    # Creates a command that runs essentially 'xacro xacro_file prefix:='
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pkg_hitw_description, "urdf", "hitw_arm_macro.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )

    robot_description = {'robot_description': robot_description_content}
    robot_params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}

    # Loads the controllers.yaml file
    robot_controllers = PathJoinSubstitution(
        [pkg_hitw_controllers, "config", "my_controllers.yaml"]
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher_node = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_params]
    )

    # Spawn the robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "HITW_system_position",
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0',],
        output="screen",
    )

    # Control Node
    controller_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Robot Controller Spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        declare_urdf_model_path_cmd,
        declare_use_robot_state_pub_cmd,
        declare_use_sim_time_cmd,
        declare_prefix,

        start_world,

        robot_state_publisher_node,
        spawn_entity,
        controller_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ])