from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy',
            default_value='true',
            description='Enable joystick input'
        ),
        DeclareLaunchArgument(
            'dev',
            default_value='/dev/input/js0',
            description='Joystick device'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            parameters=[{
                'autorepeat_rate': 10.0,
                'dev': LaunchConfiguration('dev')
            }],
            output='screen'
        ),

        Node(
            package='arm4r_teleop',
            executable='arm4r_teleop_joy.py',
            name='arm4r_teleop_joy',
            output='screen',
            parameters=[{
                'joy': LaunchConfiguration('joy')
            }]
        ),
    ])