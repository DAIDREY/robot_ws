#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch参数声明
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # 获取参数值
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # 任务管理节点 (robot_task的核心)
    task_manager_node = Node(
        package='robot_task',
        executable='task_manager',
        name='task_manager',
        parameters=[
            {
                'pre_grasp_height': 0.15,      # 预抓取高度 (米)
                'grasp_height_offset': 0.02,   # 抓取高度偏移 (米)
                'lift_height': 0.10,           # 提升高度 (米)
                'place_offset_x': 0.2,         # 放置位置X偏移 (米)
                'place_offset_y': -0.2,        # 放置位置Y偏移 (米)
                'place_offset_z': 0.1,         # 放置位置Z偏移 (米)
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        # 参数声明
        use_sim_time_arg,
        
        # 任务管理节点
        task_manager_node,
        
        # 启动完成提示
        LogInfo(msg='Robot Task节点启动完成')
    ])