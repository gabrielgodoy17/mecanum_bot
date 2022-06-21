import os
from setuptools import setup
from glob import glob

package_name = 'mecanum_bot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name + '/launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name + '/description'), glob('description/*.urdf')),
        (os.path.join('share', package_name + '/config'), glob('config/*.yaml')),
        (os.path.join('share', package_name + '/config'), glob('config/*.xml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='godoygabriel07@gmail.com',
    description='4 wheel mecanum robot teleoperation, urdf description, foward and backward kinematics nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'direction_mapper = mecanum_bot.direction_mapper:main',
            'fwd_kinematics = mecanum_bot.fwd_kinematics:main',
            'inv_kinematics = mecanum_bot.inv_kinematics:main',
            'wheels_joint_update = mecanum_bot.wheels_joint_update:main',
            'spi_interface = mecanum_bot.spi_interface:main',
            'wheels_speed_broadcaster = mecanum_bot.wheel_speed_broadcaster:main'
        ],
    },
)
