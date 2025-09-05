from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='串口设备路径'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='是否使用fake硬件'
        )
    )
    
    # 机器人驱动启动
    robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_driver'),
                'launch',
                'robot_driver.launch.py'
            ])
        ]),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'use_simulation': LaunchConfiguration('use_fake_hardware'),
        }.items()
    )
    
    # MoveIt配置启动
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sr_moveit2_config'),
                'launch',
                'demo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
        }.items()
    )
    
    return LaunchDescription(declared_arguments + [
        robot_driver_launch,
        moveit_launch,
    ])