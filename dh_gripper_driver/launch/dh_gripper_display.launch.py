from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('GripperID', default_value='1'),
        DeclareLaunchArgument('GripperModel', default_value='AG95_MB'),
        DeclareLaunchArgument('Connectport', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('Baudrate', default_value='115200'),

        Node(
            package='dh_gripper_driver',
            executable='dh_gripper_driver',
            name='dh_gripper_driver',
            output='screen',
            parameters=[{
                'Gripper_ID': LaunchConfiguration('GripperID'),
                'Gripper_Model': LaunchConfiguration('GripperModel'),
                'Connect_port': LaunchConfiguration('Connectport'),
                'BaudRate': LaunchConfiguration('Baudrate')
            }]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': LaunchConfiguration('robot_description'),
                'publish_frequency': 100.0
            }]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'source_list': ['/gripper/joint_states']}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '<path_to_your_rviz_config>/urdf.rviz']
        )
    ])
