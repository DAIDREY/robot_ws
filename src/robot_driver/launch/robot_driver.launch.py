from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command
import os

def generate_launch_description():
    # 声明启动参数
    robot_description_pkg = FindPackageShare(package='robot_description').find('robot_description')
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
            'baudrate',
            default_value='115200',
            description='串口波特率'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'config_file',
            default_value='robot_driver_params.yaml',
            description='配置文件名称'
        )
    )
    
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='是否发布TF变换'
        )
    )

    def launch_setup(context, *args, **kwargs):
        # 获取参数值
        config_file = LaunchConfiguration('config_file').perform(context)
        
        
        # 配置文件路径
        config_path = PathJoinSubstitution([
            FindPackageShare('robot_driver'),
            'config',
            config_file
        ])
        
        # 机器人驱动节点
        robot_driver_node = Node(
            package='robot_driver',
            executable='seed_robot_driver_node',
            name='seed_robot_driver',
            parameters=[
                config_path,
                {
                    'serial_port': LaunchConfiguration('serial_port'),
                    'baudrate': LaunchConfiguration('baudrate'),
                }
            ],
            output='screen',
            emulate_tty=True,
        )
        # 机器人状态发布器
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', os.path.join(robot_description_pkg, 'urdf', 'seed_robot.urdf.xacro')]),
                'publish_frequency': 50.0
            }],
            condition=IfCondition(LaunchConfiguration('publish_tf'))
        )
        
        return [
            robot_driver_node,
            robot_state_publisher,
        ]
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])