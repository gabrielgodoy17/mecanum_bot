import launch
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bot_pkg_share = FindPackageShare(package='mecanum_bot').find('mecanum_bot')
    ydlidar_pkg_share = FindPackageShare(package='ydlidar').find('ydlidar')
    mpu_pkg_share = FindPackageShare(package='mpu9250driver').find('mpu9250driver')

    default_model_path = os.path.join(bot_pkg_share, 'description/mecanum_bot_description.urdf')
    ydlidar_params_file = os.path.join(ydlidar_pkg_share, 'params/ydlidar.yaml')
    mpu_params_file = os.path.join(mpu_pkg_share, 'params/mpu9250.yaml')


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        arguments=[default_model_path]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['wheels_joint_state']}]
    )

    direction_mapper_node = Node(
        package='mecanum_bot',
        executable='direction_mapper',
        name='direction_mapper'
    )

    fwd_kinematics_node = Node(
        package='mecanum_bot',
        executable='fwd_kinematics',
        name='fwd_kinematics'
    )

    inv_kinematics_node = Node(
        package='mecanum_bot',
        executable='inv_kinematics',
        name='inv_kinematics'
    )

    wheels_joint_update_node = Node(
        package='mecanum_bot',
        executable='wheels_joint_update',
        name='wheels_joint_update'
    )

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

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
    )

    mpu_node = Node(
        package='mpu9250driver',
        executable='mpu9250driver',
        name='mpu9250driver_node',
        emulate_tty=True,
        parameters=[LaunchConfiguration('mpu_params_file')]
    )


    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path,
            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(
            name='ydlidar_params_file',
            default_value=ydlidar_params_file,
            description='FPath to the ROS2 parameters file to use.'),
        DeclareLaunchArgument(
            name='mpu_params_file',
            default_value=mpu_params_file,
            description='Path to the ROS2 parameters file to use.'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        direction_mapper_node,
        fwd_kinematics_node,
        inv_kinematics_node,
        wheels_joint_update_node,
        spi_interface_node,
        ydlidar_node,
        tf2_node
    ])
