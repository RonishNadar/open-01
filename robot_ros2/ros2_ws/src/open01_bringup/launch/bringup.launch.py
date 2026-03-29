import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_file = os.path.join(
        get_package_share_directory('open01_description'),
        'urdf',
        'open01.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyS0',
        description='Serial port connected to ESP32')
    baud_arg = DeclareLaunchArgument(
        'baud', default_value='460800',
        description='Baud rate')

    serial_bridge = Node(
        package='open01_serial_bridge',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'port':          LaunchConfiguration('port'),
            'baud':          LaunchConfiguration('baud'),
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'imu_frame_id':  'imu_link',
            'publish_tf':    True,
        }]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 10.0,
        }],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        serial_bridge,
        robot_state_publisher,
    ])
