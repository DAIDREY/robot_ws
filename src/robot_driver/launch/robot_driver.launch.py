from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_ip',
            default_value='',
            description='机器人IP地址（如果使用网络连接）'
        )
    )
    
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
            'use_simulation',
            default_value='false',
            description='是否使用仿真模式'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='是否启用调试模式'
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
        use_simulation = LaunchConfiguration('use_simulation').perform(context)
        
        # 根据仿真模式选择配置文件
        if use_simulation.lower() == 'true':
            config_file = 'simulation_params.yaml'
        
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
                    'debug_mode': LaunchConfiguration('debug'),
                }
            ],
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration('use_simulation'))
        )
        
        # 仿真模式节点（如果需要）
        simulation_node = Node(
            package='robot_driver',
            executable='seed_robot_driver_node',
            name='seed_robot_driver_sim',
            parameters=[
                config_path,
                {
                    'simulation_mode': True,
                    'debug_mode': LaunchConfiguration('debug'),
                }
            ],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('use_simulation'))
        )
        
        # 机器人状态发布器
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': '',  # 需要从robot_description包获取
                'publish_frequency': 50.0
            }],
            condition=IfCondition(LaunchConfiguration('publish_tf'))
        )
        
        return [
            robot_driver_node,
            simulation_node,
            robot_state_publisher,
        ]
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])