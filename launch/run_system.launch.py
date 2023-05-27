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

    ns='interceptor'


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

    enu_frame= {'odom_frame' : 'local_pose_ENU'}
    base_link= {'baselink_frame' : 'base_link'}
    tf_period= {'tf_pub_period' : 0.02}
    ## TODO Need to add parameter to enable/disable tf
    ## This is important to avoid conflicts with the TF published by the SLAM system
    ##  
    # px4_ros_node = Node(
    #     package='px4_ros_com',
    #     executable='px4_ros',
    #     output='screen',
    #     name=ns+'_px4_ros_com',
    #     namespace=ns,
    #     parameters=[enu_frame, base_link, tf_period],
    #     remappings=[('vio/ros_odom', 'vio/ros_odom')]
    # )

    # Static TF base_link -> depth_camera
    # .15 0 .25 0 0 1.5707
    cam_x = 0.15
    cam_y = 0.0
    cam_z = 0.25
    cam_roll = radians(-90.0)
    cam_pitch = 0.0
    cam_yaw = radians(-90.0)
    cam_tf_node = Node(
        package='tf2_ros',
        name=ns+'_base2depth_tf_node',
        executable='static_transform_publisher',
        # arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/'+base_link['child_frame'], ns+'/depth_camera'],
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), ns+'/'+base_link['baselink_frame'], 'x500_d435_1/link/realsense_d435'],
        
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
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolov8_bringup'),
                'launch/yolov8.launch.py'
            ])
        ]),
        launch_arguments={
            'model': '/home/user/shared_volume/ros2_ws/src/d2dtracker_drone_detector/config/drone_detection_v3.pt',
            'threshold' : '0.5',
            'input_image_topic' : '/interceptor/image',
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
            'depth_topic': 'interceptor/depth_image',
            'debug' : 'false',
            'caminfo_topic' : 'interceptor/camera_info',
            'detections_poses_topic': 'yolo_detections_poses',
            'yolo_detections_topic': 'detections',
            'detector_ns' : '',
            'reference_frame' : 'map'
        }.items()
    )


    ld.add_action(cam_tf_node)
    ld.add_action(xrce_agent_launch)
    ld.add_action(kf_launch)
    # ld.add_action(px4_ros_node)
    ld.add_action(predictor_launch)
    ld.add_action(yolov8_launch)
    ld.add_action(yolo2pose_launch)

    return ld