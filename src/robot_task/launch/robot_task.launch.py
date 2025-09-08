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
        
        # 启动信息
        LogInfo(msg='🤖 启动Robot Task节点...'),
        
        # 任务管理节点
        task_manager_node,
        
        # 启动完成提示
        LogInfo(msg='✅ Robot Task节点启动完成！'),
        LogInfo(msg=''),
        LogInfo(msg='📋 此节点提供以下接口:'),
        LogInfo(msg='  服务: /grasp_object'),
        LogInfo(msg='  发布: /robot_task/status'),
        LogInfo(msg='  订阅: /robot_visioner/center_point'),
        LogInfo(msg='  发布: /pose_target, /gripper_command'),
        LogInfo(msg=''),
        LogInfo(msg='🎯 使用方法:'),
        LogInfo(msg='  ros2 service call /grasp_object robot_task/srv/GraspObject "{object_name: \'cup\'}"'),
        LogInfo(msg=''),
        LogInfo(msg='📊 监控状态:'),
        LogInfo(msg='  ros2 topic echo /robot_task/status'),
        LogInfo(msg=''),
        LogInfo(msg='🔧 系统就绪，等待其他节点和抓取指令...')
    ])