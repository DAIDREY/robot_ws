#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """生成Robot Visioner系统的launch描述（完整修复版）"""
    
    # 包路径
    pkg_robot_visioner = FindPackageShare('robot_visioner')
    
    # 配置文件路径
    yolo_config_file = PathJoinSubstitution([
        pkg_robot_visioner, 'config', 'enhanced_yolo_config.yaml'
    ])
    
    robot_visioner_config_file = PathJoinSubstitution([
        pkg_robot_visioner, 'config', 'robot_visioner_config.yaml'
    ])
    
    # Launch参数声明
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='相机名称前缀'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    yolo_model_path_arg = DeclareLaunchArgument(
        'yolo_model_path',
        default_value='/home/ubuntu/robot_ws/src/robot_visioner/models/best.pt',
        description='YOLO分割模型路径'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='检测置信度阈值'
    )
    
    enable_segmentation_arg = DeclareLaunchArgument(
        'enable_segmentation',
        default_value='true',
        description='是否启用分割功能'
    )
    
    enable_clustering_arg = DeclareLaunchArgument(
        'enable_clustering',
        default_value='true',
        description='是否启用点云聚类分析'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_depth_optical_frame',
        description='参考坐标系'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='YOLO推理设备 (cpu/cuda)'
    )
    
    target_classes_arg = DeclareLaunchArgument(
        'target_classes',
        default_value='',
        description='目标检测类别，例如："0,1,2" 或空字符串表示所有类别'
    )
    
    # 获取launch配置
    camera_name = LaunchConfiguration('camera_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    enable_segmentation = LaunchConfiguration('enable_segmentation')
    enable_clustering = LaunchConfiguration('enable_clustering')
    frame_id = LaunchConfiguration('frame_id')
    device = LaunchConfiguration('device')
    target_classes = LaunchConfiguration('target_classes')
    
    # 增强版YOLO检测节点
    enhanced_yolo_node = Node(
        package='robot_visioner',
        executable='yolo_detector',
        name='yolo_detector',
        parameters=[
            yolo_config_file,  # 加载enhanced_yolo_config.yaml配置文件
            {
                # 覆盖配置文件中的特定参数（使用正确的话题名称）
                'model_path': yolo_model_path,
                'input_topic': '/camera/color/image_raw',           # 正确的RGB话题
                'camera_info_topic': '/camera/color/camera_info',   # 正确的RGB相机信息话题
                'output_detection_topic': '/yolo/detection_result',
                'output_mask_topic': '/yolo/mask',
                'output_combined_topic': '/yolo/combined_result',
                'confidence_threshold': confidence_threshold,
                'enable_segmentation': enable_segmentation,
                'target_classes': target_classes,
                'device': device,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )
    
    # Robot Visioner主节点
    robot_visioner_node = Node(
        package='robot_visioner',
        executable='robot_visioner',
        name='robot_visioner',
        parameters=[
            robot_visioner_config_file,  # 加载robot_visioner_config.yaml配置文件
            {
                # 覆盖配置文件中的特定参数（使用正确的话题名称）
                'depth_topic': '/camera/depth/image_raw',           # 深度图话题
                'mask_topic': '/yolo/mask',                         # 来自YOLO的mask
                'camera_info_topic': '/camera/depth/camera_info',   # 深度相机信息话题
                'output_topic': '/extracted_pointcloud',
                'frame_id': frame_id,
                'enable_clustering': enable_clustering,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )
    
    return LaunchDescription([
        # Launch参数
        camera_name_arg,
        use_sim_time_arg,
        yolo_model_path_arg,
        confidence_threshold_arg,
        enable_segmentation_arg,
        enable_clustering_arg,
        frame_id_arg,
        device_arg,
        target_classes_arg,
        
        # 启动信息
        LogInfo(msg='🚀 启动Robot Visioner系统（完整修复版）...'),
        LogInfo(msg=['📷 相机前缀: ', camera_name]),
        LogInfo(msg=['🎯 YOLO模型: ', yolo_model_path]),
        LogInfo(msg=['🔧 设备: ', device]),
        LogInfo(msg=['📊 坐标系: ', frame_id]),
        LogInfo(msg=['🧩 分割功能: ', enable_segmentation]),
        LogInfo(msg=['🔍 聚类分析: ', enable_clustering]),
        LogInfo(msg='📋 正确的话题映射:'),
        LogInfo(msg='  RGB图像: /camera/color/image_raw'),
        LogInfo(msg='  深度图像: /camera/depth/image_raw'),
        LogInfo(msg='  RGB相机信息: /camera/color/camera_info'),
        LogInfo(msg='  深度相机信息: /camera/depth/camera_info'),
        LogInfo(msg='📤 输出话题:'),
        LogInfo(msg='  YOLO检测结果: /yolo/detection_result'),
        LogInfo(msg='  分割Mask: /yolo/mask'),
        LogInfo(msg='  合并结果: /yolo/combined_result'),
        LogInfo(msg='  提取点云: /extracted_pointcloud'),
        LogInfo(msg='  中心点: /robot_visioner/center_point'),
        LogInfo(msg='  可视化标记: /robot_visioner/center_marker'),
        
        # 节点
        enhanced_yolo_node,
        robot_visioner_node,
        
        LogInfo(msg='✅ Robot Visioner系统启动完成'),
        LogInfo(msg='💡 检查数据流命令:'),
        LogInfo(msg='  ros2 topic hz /yolo/mask'),
        LogInfo(msg='  ros2 topic hz /extracted_pointcloud'),
        LogInfo(msg='  ros2 topic hz /robot_visioner/center_point'),
    ])