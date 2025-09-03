import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import conditions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare(package='robot_description').find('robot_description')
    
    # Path to URDF file
    default_model_path = os.path.join(pkg_share, 'urdf/seed_robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    
    # Launch configuration variables
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rvizconfig')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')
    
    # Declare launch arguments
    declare_model_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_gui_cmd = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Flag to enable RViz'
    )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', os.path.join(pkg_share, 'urdf', 'seed_robot.urdf.xacro')]),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=conditions.IfCondition(use_gui)
    )
    
    # Joint State Publisher node (fallback when GUI is disabled)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=conditions.UnlessCondition(use_gui)
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=conditions.IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create launch description and populate
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_model_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_gui_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
    
    return ld