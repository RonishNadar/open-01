from setuptools import setup

package_name = 'open01_serial_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ronish',
    maintainer_email='you@example.com',
    description='Serial bridge between ESP32 and ROS2 for OPEN-01',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge = open01_serial_bridge.serial_bridge_node:main',
        ],
    },
)