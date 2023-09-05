#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
    ld = LaunchDescription()

    map2global_tf_node = Node(
        package='tf2_ros',
        name='map2global_tf',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'global'],
        
    )

    global2odom_tf_node = Node(
        package='tf2_ros',
        name='map2global_tf',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'global', 'odom'],
    )

    imu2baselink_tf_node = Node(
        package='tf2_ros',
        name='imu2baselink_tf',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'imu', 'base_link'],
    )

    topic_relay_node = Node(
        package='topic_tools',
        name='odom_topic_relay',
        executable='relay',
        arguments=['/ov_msckf/odomimu', '/mavros/odometry/out'],
    )

    ld.add_action(map2global_tf_node)
    ld.add_action(global2odom_tf_node)
    ld.add_action(imu2baselink_tf_node)
    ld.add_action(topic_relay_node)

    return ld