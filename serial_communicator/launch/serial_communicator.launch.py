from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch引数の宣言
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='Serial port device'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='57600',
            description='Serial port baudrate'
        ),
        
        # ノードの起動
        Node(
            package='serial_communicator',
            executable='serial_communicator_node',
            name='serial_communicator',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port')},
                {'baudrate': LaunchConfiguration('baudrate')},
                {'angle_topic_name': 'arm_angles'}
            ]
        )
    ])