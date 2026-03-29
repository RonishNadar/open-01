from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
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

    return LaunchDescription([
        port_arg,
        baud_arg,
        serial_bridge,
    ])
