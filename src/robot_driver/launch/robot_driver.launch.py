from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明启动参数
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
        
        return [robot_driver_node]
    
    return LaunchDescription(declared_arguments + [
        OpaqueFunction(function=launch_setup)
    ])