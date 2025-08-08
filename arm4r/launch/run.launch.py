from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch argument to control RViz startup
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz2'
    )
    
    # Launch argument for serial port
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/serial/by-id/usb-ROBOTIS_ROBOTIS_ComPort-if00',
        description='Serial port device'
    )

    # Launch argument for baudrate
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial port baudrate'
    )

    serial_communicator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('serial_communicator'),
                'launch',
                'serial_communicator.launch.py'
            )
        ),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate')
        }.items()
    )

    arm_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arm_controller'),
                'launch',
                'arm_controller.launch.py'
            )
        )
    )

    rviz_config = os.path.join(
        get_package_share_directory('arm_controller'),
        'rviz',
        'rviz.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        port_arg,
        baudrate_arg,
        serial_communicator_launch,
        arm_controller_launch,
        rviz_node
    ])