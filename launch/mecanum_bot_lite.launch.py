import launch
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    bot_pkg_share = FindPackageShare(package='mecanum_bot').find('mecanum_bot')
    ydlidar_pkg_share = FindPackageShare(package='ydlidar').find('ydlidar')
    #ydlidar_pkg_share = FindPackageShare(package='ydlidar_ros2_driver').find('ydlidar_ros2_driver')
    # mpu_pkg_share = FindPackageShare(package='mpu9250driver').find('mpu9250driver')

    default_model_path = os.path.join(bot_pkg_share, 'description/mecanum_bot_description.urdf')
    ydlidar_params_file = os.path.join(ydlidar_pkg_share, 'params/ydlidar.yaml')
    # mpu_params_file = os.path.join(mpu_pkg_share, 'params/mpu9250.yaml')

    spi_interface_node = Node(
        package='mecanum_bot',
        executable='spi_interface',
        name='spi_interface'
    )    
    
    ydlidar_node = Node(
        package='ydlidar',
        executable='ydlidar_node',
        name='ydlidar_node',
        emulate_tty=True,
        parameters=[LaunchConfiguration('ydlidar_params_file')],
    )

    # ydlidar_node = LifecycleNode(package='ydlidar_ros2_driver',
    #                         node_executable='ydlidar_ros2_driver_node',
    #                         node_name='ydlidar_ros2_driver_node',
    #                         output='screen',
    #                         emulate_tty=True,
    #                         parameters=[LaunchConfiguration('ydlidar_params_file')],
    #                         node_namespace='/',
    #                         )

    # mpu_node = Node(
    #     package='mpu9250driver',
    #     executable='mpu9250driver',
    #     name='mpu9250driver_node',
    #     emulate_tty=True,
    #     parameters=[LaunchConfiguration('mpu_params_file')]
    # )


    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path,
            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            name='ydlidar_params_file',
            default_value=ydlidar_params_file,
            description='FPath to the ROS2 parameters file to use.'),
        # DeclareLaunchArgument(
        #     name='mpu_params_file',
        #     default_value=mpu_params_file,
        #     description='Path to the ROS2 parameters file to use.'),
        spi_interface_node,
        ydlidar_node
    ])
