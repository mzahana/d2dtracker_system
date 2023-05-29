#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from math import radians

def generate_launch_description():
    ld = LaunchDescription()

    ns=''


    # MicroXRCEAgent
    xrce_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_system'),
                'xrce_agent.launch.py'
            ])
        ]),
        launch_arguments={
            'interface': 'serial',
            'device': '/dev/ttyUSB0',
            'baud': '921600'
        }.items()
    )

    # isaac_visula_slam + realsense
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_system'),
                'slam_realsense.launch.py'
            ])
        ])
    )

    
    px4_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('px4_ros_com'),
                'launch/px4_ros_tf.launch.py'
            ])
        ]),
        launch_arguments={
            'ns': '',
            'odom_frame': 'odom',
            'base_link' : 'px4_base_link',
            'tf_period' : '0.02',
            'publish_tf': 'True'
        }.items()
    )

    # Static TF base_link -> depth_camera
    # .15 0 .25 0 0 1.5707
    cam_x = 0.1
    cam_y = 0.0
    cam_z = 0.0
    cam_roll = radians(-90.0)
    cam_pitch = 0.0
    cam_yaw = radians(-90.0)
    cam_tf_node = Node(
        package='tf2_ros',
        name='base2depth_tf_node',
        executable='static_transform_publisher',
        # arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/'+base_link['child_frame'], ns+'/depth_camera'],
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/base_link', 'camera_link'],
        
    )

    # Kalman filter
    kf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('multi_target_kf'),
                'launch/kf_const_vel.launch.py'
            ])
        ]),
        launch_arguments={
            'detections_topic': 'yolo_detections_poses',
            'kf_ns' : ''
        }.items()
    )

    # Trajectory prediction
    predictor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('trajectory_prediction'),
                'launch/predictor.launch.py'
            ])
        ]),
        launch_arguments={
            'kf_topic': 'kf/good_tracks',
            'namespace' : '',
            'log_level' : 'info'
        }.items()
    )
    
    # yolov8_ros
    package_name = 'd2dtracker_drone_detector'
    file_name = 'drone_detection_v3.pt'
    package_share_directory = get_package_share_directory(package_name)
    file_path = os.path.join(package_share_directory, file_name)
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolov8_bringup'),
                'launch/yolov8.launch.py'
            ])
        ]),
        launch_arguments={
            'model': file_path,
            'threshold' : '0.5',
            'input_image_topic' : '/camera/color/image_raw',
            'device': 'cuda:0'
        }.items()
    )

    # Yolo to pose node
    yolo2pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_drone_detector'),
                'yolo2pose.launch.py'
            ])
        ]),
        launch_arguments={
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'debug' : 'false',
            'caminfo_topic' : '/camera/aligned_depth_to_color/camera_info',
            'detections_poses_topic': 'yolo_detections_poses',
            'yolo_detections_topic': 'detections',
            'detector_ns' : '',
            'reference_frame' : 'map'
        }.items()
    )


    ld.add_action(cam_tf_node)
    ld.add_action(xrce_agent_launch)
    ld.add_action(kf_launch)
    ld.add_action(predictor_launch)
    ld.add_action(yolov8_launch)
    ld.add_action(yolo2pose_launch)
    ld.add_action(px4_ros_launch)
    ld.add_action(slam_launch)

    return ld