from setuptools import setup
import os
from glob import glob

package_name = 'launch_tests_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'rviz_config'), glob('rviz_config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot_exe = launch_tests_pkg.move_robot:main',
            'move_robot_with_arguments_exe = launch_tests_pkg.move_robot_with_arguments:main',
            'move_robot_with_params_exe = launch_tests_pkg.move_robot_with_params:main',
        ],
    },
)