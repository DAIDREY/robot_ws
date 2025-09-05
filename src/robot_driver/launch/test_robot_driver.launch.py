# launch/test_robot_driver.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'test_type',
            default_value='basic',
            description='测试类型: basic, performance, integration'
        )
    )
    
    # 基础功能测试节点
    test_basic_node = Node(
        package='robot_driver',
        executable='test_basic_functions',
        name='test_basic_functions',
        output='screen',
        parameters=[{
            'test_duration': 30.0,  # 测试持续时间（秒）
            'test_commands': ['connect', 'move_joint', 'move_linear', 'disconnect']
        }]
    )
    
    # 性能测试节点
    test_performance_node = Node(
        package='robot_driver',
        executable='test_performance',
        name='test_performance',
        output='screen',
        parameters=[{
            'command_count': 1000,
            'test_frequency': 100.0  # Hz
        }]
    )
    
    # 运行单元测试
    run_unit_tests = ExecuteProcess(
        cmd=['colcon', 'test', '--packages-select', 'robot_driver'],
        output='screen'
    )
    
    return LaunchDescription(declared_arguments + [
        test_basic_node,
        # test_performance_node,  # 根据需要启用
        # run_unit_tests,  # 根据需要启用
    ])