from setuptools import setup
import os
from glob import glob

package_name = 'open01_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ronish',
    maintainer_email='you@example.com',
    description='Launch files for OPEN-01',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)