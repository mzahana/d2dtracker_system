#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from math import radians

def generate_launch_description():
    ld = LaunchDescription()

    ns=''


    # MicroXRCEAgent
    run_xrce = LaunchConfiguration('run_xrce')
    run_xrce_launch_arg = DeclareLaunchArgument(
        'run_xrce',
        default_value=os.environ.get('RUN_XRCE', 'False'),
    )
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
        }.items(),
        condition=LaunchConfigurationEquals('run_xrce', 'True')
    )

    # Realsense
    run_rs = LaunchConfiguration('run_rs')
    run_rs_launch_arg = DeclareLaunchArgument(
        'run_rs',
        default_value=os.environ.get('RUN_REALSENSE', 'False'),
    )
    
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_system'),
                'realsense.launch.py'
            ])
        ]),
        condition=LaunchConfigurationEquals('run_rs', 'True')
    )

    # isaac_visual_slam + realsense
    # NOTE This will be rnu seprately, in a different launch file
    # run_slam = LaunchConfiguration('run_slam')
    # run_slam_launch_arg = DeclareLaunchArgument(
    #     'run_slam',
    #     default_value=os.environ.get('RUN_SLAM', 'False'),
    # )
    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('d2dtracker_system'),
    #             'slam_realsense.launch.py'
    #         ])
    #     ]),
    #     condition=LaunchConfigurationEquals('run_slam', 'True')
    # )

    run_vins = LaunchConfiguration('run_vins')
    run_vins_launch_arg = DeclareLaunchArgument(
        'run_vins',
        default_value=os.environ.get('RUN_OPENVINS', 'False'),
    )
    vins_config = LaunchConfiguration('vins_config')
    vins_config_launch_arg = DeclareLaunchArgument(
        'vins_config',
        default_value='/home/d2d/shared_volume/ros2_ws/src/open_vins/config/d455_custom/estimator_config.yaml',
    )
    vins_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ov_msckf'),
                'launch/subscribe.launch.py'
            ])
        ]),
        launch_arguments={
            'config_path': vins_config
        }.items(),
        condition=LaunchConfigurationEquals('run_vins', 'True')
    )

    
    run_px4_ros = LaunchConfiguration('run_px4_ros')
    run_px4_launch_arg = DeclareLaunchArgument(
        'run_px4_ros',
        default_value=os.environ.get('RUN_PX4_ROS', 'False'),
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
            'publish_tf': 'True',
            'vio_topic' : '/visual_slam/tracking/odometry'
        }.items(),
        condition=LaunchConfigurationEquals('run_px4_ros', 'True')
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
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), 'base_link', 'camera_link'],
        
    )

    # Kalman filter
    run_kf = LaunchConfiguration('run_kf')
    run_kf_launch_arg = DeclareLaunchArgument(
        'run_kf',
        default_value=os.environ.get('RUN_KF', 'False'),
    )
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
        }.items(),
        condition=LaunchConfigurationEquals('run_kf', 'True')
    )

    # Trajectory prediction
    run_traj_pred = LaunchConfiguration('run_traj_pred')
    run_traj_pred_launch_arg = DeclareLaunchArgument(
        'run_traj_pred',
        default_value=os.environ.get('RUN_TRAJ_PRED', 'False'),
    )
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
        }.items(),
        condition=LaunchConfigurationEquals('run_traj_pred', 'True')
    )
    
    # yolov8_ros
    run_yolo = LaunchConfiguration('run_yolo')
    run_yolo_launch_arg = DeclareLaunchArgument(
        'run_yolo',
        default_value=os.environ.get('RUN_YOLO', 'False'),
    )
    package_name = 'd2dtracker_drone_detector'
    file_name = 'drone_detection_v3.pt'
    package_share_directory = get_package_share_directory(package_name)
    file_path = os.path.join(package_share_directory, file_name)
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_system'),
                'yolov8.launch.py'
            ])
        ]),
        launch_arguments={
            'model': file_path,
            'threshold' : '0.5',
            'input_image_topic' : '/camera/color/image_raw',
            'device': 'cuda:0'
        }.items(),
        condition=LaunchConfigurationEquals('run_yolo', 'True')
    )

    # Yolo to pose node
    run_yolo_pose = LaunchConfiguration('run_yolo_pose')
    run_yolo_pose_launch_arg = DeclareLaunchArgument(
        'run_yolo_pose',
        default_value=os.environ.get('RUN_YOLO_POSE', 'False'),
    )
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
            'yolo_detections_topic': 'yolo/detections',
            'detector_ns' : '',
            'reference_frame' : 'map'
        }.items(),
        condition=LaunchConfigurationEquals('run_yolo_pose', 'True')
    )

    run_arducam_stereo = LaunchConfiguration('run_arducam_stereo')
    run_arducam_stereo_launch_arg = DeclareLaunchArgument(
        'run_arducam_stereo',
        default_value=os.environ.get('RUN_ARDUCAM_STEREO', 'False'),
    )
    arducam_stereo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arducam_ros2'),
                'stereo.launch.py'
            ])
        ]),
        condition=LaunchConfigurationEquals('run_arducam_stereo', 'True')
    )

    # MAVROS
    # 'fcu_url': 'udp://:14540@192.168.0.4:14540' for ethernet connection  (Pixhawk 6X)
    # 'gcs_url':'udp://:14550@192.168.1.79:14550'
    run_mavros = LaunchConfiguration('run_mavros')
    run_mavros_launch_arg = DeclareLaunchArgument(
        'run_mavros',
        default_value=os.environ.get('RUN_MAVROS', 'False'),
    )
    
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_system'),
                'mavros.launch.py'
            ])
        ]),
        launch_arguments={
            'fcu_url': '/dev/ttyUSB0:921600',
            'gcs_url':'udp://:14550@192.168.1.151:14550',
            'pluginlists_yaml': '/home/d2d/shared_volume/ros2_ws/src/d2dtracker_system/config/px4_pluginlists.yaml'
        }.items(),
        condition=LaunchConfigurationEquals('run_mavros', 'True')
    )

    # Static TFs required by MAVROS to fuse visual odometry
    run_mavros_tfs = LaunchConfiguration('run_mavros_tfs')
    run_mavros_tfs_launch_arg = DeclareLaunchArgument(
        'run_mavros_tfs',
        default_value=os.environ.get('RUN_MAVROS_TFS', 'False'),
    )
    
    mavros_tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_system'),
                'mavros_static_tfs.launch.py'
            ])
        ]),
        condition=LaunchConfigurationEquals('run_mavros_tfs', 'True')
    )


    ld.add_action(run_rs_launch_arg)
    ld.add_action(realsense_launch)

    ##This is for isaac_ros_visual_slam (NOT USED)
    # ld.add_action(run_slam_launch_arg)
    # ld.add_action(slam_launch)

    ld.add_action(vins_config_launch_arg)
    ld.add_action(run_vins_launch_arg)
    ld.add_action(vins_launch)

    ld.add_action(run_xrce_launch_arg)
    ld.add_action(xrce_agent_launch)

    ld.add_action(run_px4_launch_arg)
    ld.add_action(px4_ros_launch)

    ld.add_action(run_kf_launch_arg)
    ld.add_action(kf_launch)

    ld.add_action(run_traj_pred_launch_arg)
    ld.add_action(predictor_launch)

    ld.add_action(run_yolo_launch_arg)
    ld.add_action(yolov8_launch)

    ld.add_action(run_yolo_pose_launch_arg)
    ld.add_action(yolo2pose_launch)

    ld.add_action(run_arducam_stereo_launch_arg)
    ld.add_action(arducam_stereo_launch)
    
    ld.add_action(cam_tf_node)

    ld.add_action(run_mavros_launch_arg)
    ld.add_action(mavros_launch)

    ld.add_action(run_mavros_tfs_launch_arg)
    ld.add_action(mavros_tfs_launch)
    
    return ld