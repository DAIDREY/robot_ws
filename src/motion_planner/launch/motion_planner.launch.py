from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # 获取包路径
    pkg_motion_planner = get_package_share_directory('motion_planner')
    
    # 参数文件路径
    params_file = PathJoinSubstitution([
        FindPackageShare('motion_planner'),
        'config',
        'motion_planner_params.yaml'
    ])
    
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=params_file,
        description='Full path to config file'
    )
    
    # 获取参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    
    # 运动规划节点
    motion_planner_node = Node(
        package='motion_planner',
        executable='motion_planner_node',
        name='motion_planner_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # 重映射话题到robot_driver的接口
            ('planned_trajectory', '/joint_trajectory'),
            ('joint_states', '/joint_states'),
            ('current_pose', '/current_pose')
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        motion_planner_node
    ])