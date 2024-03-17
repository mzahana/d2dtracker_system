import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode

PACKAGE_NAME = 'apriltag_ros'

def generate_launch_description():
    # Configs files
    defult_config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "params.yaml")
    defult_tags_config_file = os.path.join(get_package_share_directory(PACKAGE_NAME), "config", "tags.yaml")
    params_yaml_arg = DeclareLaunchArgument(
        name='params_yaml', 
        default_value = defult_config_file, 
        description='Parameters of apriltag node')
    
    params_yaml = LaunchConfiguration('params_yaml')

    tags_yaml_arg = DeclareLaunchArgument(
        name='tags_yaml', 
        default_value = defult_tags_config_file, 
        description='Tag Parameters')
    
    tags_yaml = LaunchConfiguration('tags_yaml')

    # Launch arguments
    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time', 
        default_value='False', 
        choices=['True', 'False'],
        description='Parameter use_sim_time')

    # Launch arguments
    sim_time = LaunchConfiguration('use_sim_time')

    # Nodes to launch
    node = Node(
        package=PACKAGE_NAME,
        executable='ContinuousDetector', # Other option SingleDetector
        arguments=['--ros-args', '--log-level', 'info'],
        parameters=[params_yaml],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        GroupAction(
            actions=[
                sim_time_arg,
                params_yaml_arg,
                tags_yaml_arg,
                SetParameter("use_sim_time", sim_time),
                SetParameter("tags_yaml_path", tags_yaml),
                node,
            ]
        )
    ])