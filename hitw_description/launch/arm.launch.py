import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
 
    # Setting default paths
    pkg_hitw_description = get_package_share_directory('hitw_description')
    pkg_hitw_controllers = get_package_share_directory('hitw_controllers')
    pkg_hitw_hardware = get_package_share_directory('hitw_hardware')
    default_rviz_config_path = os.path.join(pkg_hitw_description, 'rviz/urdf_config.rviz')
    default_urdf_model_path = os.path.join(pkg_hitw_description, 'urdf/hitw_arm_macro.urdf.xacro')
    
    # Launch configuration variables specific to simulation
    urdf_model = LaunchConfiguration('urdf_model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')
    
    # Declare the launch arguments  
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file'
    )
        
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='false',
        description='Whether to start Gazebo'
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
            " ",
            "use_gazebo:=",
            use_gazebo,
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Loads the controllers.yaml file
    robot_controllers = PathJoinSubstitution(
        [pkg_hitw_controllers, "config", "my_controllers.yaml"]
    )
    
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_publisher_node = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
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
    
    # Launch RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    ld.add_action(declare_use_rviz_cmd) 
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_prefix)
    
    # Add any actions
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(controller_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(robot_controller_spawner)

    
    return ld