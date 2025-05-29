from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('Gripper_ID', default_value='1'),
        DeclareLaunchArgument('Gripper_Model', default_value='PGC140'),
        DeclareLaunchArgument('Connect_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('BaudRate', default_value='115200'),
        DeclareLaunchArgument('test_run', default_value='true'),

        Node(
            package='dh_gripper_driver',
            executable='dh_gripper_driver',
            name='dh_gripper_driver',
            output='screen',
            parameters=[{
                'Gripper_ID': LaunchConfiguration('Gripper_ID'),
                'Gripper_Model': LaunchConfiguration('Gripper_Model'),
                'Connect_port': LaunchConfiguration('Connect_port'),
                'BaudRate': LaunchConfiguration('BaudRate')
            }]
        ),

        Node(
            package='dh_gripper_driver',
            executable='dh_gripper_driver_test',
            name='dh_gripper_driver_test',
            output='screen',
            parameters=[{
                'Gripper_Model': LaunchConfiguration('Gripper_Model')
            }],
            condition=IfCondition(LaunchConfiguration('test_run'))
        )
    ])
