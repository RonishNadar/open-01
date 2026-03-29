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

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyLIDAR',
        description='Serial port connected to LDS-02 lidar')

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

    lidar = Node(
        package='open01_serial_bridge',
        executable='lidar',
        name='lidar',
        output='screen',
        parameters=[{
            'port':      LaunchConfiguration('lidar_port'),
            'frame_id':  'laser',
            'min_range': 0.12,
            'max_range': 3.5,
        }]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        lidar_port_arg,
        serial_bridge,
        lidar,
    ])
