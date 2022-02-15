import launch
import launch_ros
import os
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='mecanum_bot').find('mecanum_bot')
    default_model_path = os.path.join(pkg_share, 'description/mecanum_bot_description.urdf')

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

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml')]
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

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                             description='Absolute path to robot urdf file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        robot_localization_node,
        direction_mapper_node,
        fwd_kinematics_node,
        wheels_joint_update_node,
	spi_interface_node
    ])
