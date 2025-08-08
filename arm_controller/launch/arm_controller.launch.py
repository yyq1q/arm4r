from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'angle_topic_name',
            default_value='/arm_angles',
            description='Topic name for arm angles'
        ),
        
        Node(
            package='arm_controller',
            executable='test_node',
            name='arm_controller',
            parameters=[{
                'angle_topic_name': LaunchConfiguration('angle_topic_name')
            }],
            output='screen'
        )
    ])