from setuptools import setup
import os
from glob import glob

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Declare config and launch files paths
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raymundo',
    maintainer_email='raymundo@todo.todo',
    description='Motor control package with PID controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dc_motor = motor_control.dc_motor:main',
            'set_point = motor_control.set_point:main',
            'controller = motor_control.controller:main' 
        ],
    },
)