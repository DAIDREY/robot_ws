#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import sys

def generate_launch_description():
    # 获取包的路径
    pkg_share = FindPackageShare('robot_visioner').find('robot_visioner')
    
    # 设置Python路径环境变量，确保使用conda环境
    python_path_action = SetEnvironmentVariable(
        'PYTHONPATH',
        os.environ.get('PYTHONPATH', '')
    )
    
    # 声明启动参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/ubuntu/robot_ws/src/robot_visioner/models/best.pt',
        description='Path to YOLO model file (absolute or relative to package)'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for object detection'
    )
    
    iou_threshold_arg = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.4',
        description='IoU threshold for NMS'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/color/image_raw',
        description='Input image topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/yolov8/image',
        description='Output annotated image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/color/camera_info',
        description='Camera info topic'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to run inference on (cpu or cuda)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'yolo_config.yaml']),
        description='Path to configuration file'
    )
    
    # YOLO检测节点
    yolo_detector_node = Node(
        package='robot_visioner',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'iou_threshold': LaunchConfiguration('iou_threshold'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'device': LaunchConfiguration('device'),
        }],
        output='screen',
        emulate_tty=True,
        # 添加环境变量，确保使用正确的Python路径
        additional_env={'PYTHONPATH': os.environ.get('PYTHONPATH', '')},
        # 如果需要，也可以指定Python可执行文件
        # prefix=f'{sys.executable} -m',
    )
    
    return LaunchDescription([
        python_path_action,
        model_path_arg,
        confidence_threshold_arg,
        iou_threshold_arg,
        input_topic_arg,
        output_topic_arg,
        camera_info_topic_arg,
        device_arg,
        config_file_arg,
        LogInfo(msg=['Starting YOLO Detector with model: ', LaunchConfiguration('model_path')]),
        LogInfo(msg=['Using Python executable: ', sys.executable]),
        LogInfo(msg=['PYTHONPATH: ', os.environ.get('PYTHONPATH', 'Not set')]),
        yolo_detector_node,
    ])