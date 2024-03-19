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
    # run_xrce = LaunchConfiguration('run_xrce')
    # run_xrce_launch_arg = DeclareLaunchArgument(
    #     'run_xrce',
    #     default_value=os.environ.get('RUN_XRCE', 'False'),
    # )
    # xrce_agent_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('d2dtracker_system'),
    #             'xrce_agent.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'interface': 'serial',
    #         'device': '/dev/ttyUSB0',
    #         'baud': '921600'
    #     }.items(),
    #     condition=LaunchConfigurationEquals('run_xrce', 'True')
    # )

    #
    # Realsense
    #
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

    #
    # open_vins
    #
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
            'config_path': os.environ.get('OPENVINS_YAML', '')
        }.items(),
        condition=LaunchConfigurationEquals('run_vins', 'True')
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

    #
    # Kalman filter
    #
    run_kf = LaunchConfiguration('run_kf')
    run_kf_launch_arg = DeclareLaunchArgument(
        'run_kf',
        default_value= os.environ.get('RUN_KF', 'False'),
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
            'kf_ns' : '',
            'kf_yaml': os.environ.get('KF_YAML', '')
        }.items(),
        condition=LaunchConfigurationEquals('run_kf', 'True')
    )

    #
    # Trajectory prediction
    #
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
    
    #
    # yolov8_ros
    #
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
            'model': os.environ.get('YOLOV8_MODEL_PATH', '') ,
            'threshold' : '0.5',
            'input_image_topic' : '/camera/color/image_raw',
            'device': 'cuda:0'
        }.items(),
        condition=LaunchConfigurationEquals('run_yolo', 'True')
    )

    #
    # Yolo to pose node
    #
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

    #
    # Arducam stereo
    #
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

    
    #
    # MAVROS
    #
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
            'fcu_url': os.environ.get('MAVROS_FCU_URL', ''),
            'gcs_url':os.environ.get('MAVROS_GCS_URL', ''),
            'pluginlists_yaml': os.environ.get('MAVROS_PLUGINLIST_YAML', ''),
            'tgt_system': os.environ.get('MAVLINK_ID', '1')
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

    #
    # apriltag_ros
    #
    run_apriltag = LaunchConfiguration('run_apriltag')
    run_apriltag_launch_arg = DeclareLaunchArgument(
        'run_apriltag',
        default_value=os.environ.get('RUN_APRILTAG', 'False'),
    )
    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('d2dtracker_system'),
                'apriltag_ros.launch.py'
            ])
        ]),
        launch_arguments={
            'params_yaml': os.environ.get('APRILTAG_PARAMS_YAML', ''),
            'tags_yaml': os.environ.get('APRILTAG_TAGS_YAML', '')
        }.items(),
        condition=LaunchConfigurationEquals('run_apriltag', 'True')
    )
    apriltag_tools_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('apriltag_tools_ros'),
                'apriltag_detection_to_pose.launch.py'
            ])
        ]),
        launch_arguments={
            'reference_frame': os.environ.get('APRILTAG_PARENT_FRAME', '')
        }.items(),
        condition=LaunchConfigurationEquals('run_apriltag', 'True')
    )

    #
    # drone_path_predictor_ros
    #
    run_gru_path_predictor = LaunchConfiguration('run_gru_path_predictor')
    run_gru_launch_arg = DeclareLaunchArgument(
        'run_gru_path_predictor',
        default_value=os.environ.get('RUN_GRU_PATH_PREDICTOR', 'False'),
    )
    drone_path_predictor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_path_predictor_ros'),
                'launch/trajectory_predictor.launch.py'
            ])
        ]),
        launch_arguments={
            'param_file': os.environ.get('GRU_YAML_FILE', ''),
            'pose_topic': '/kf/good_tracks_pose_array',
            'path_topic': '/gru_predicted_path'
        }.items(),
        condition=LaunchConfigurationEquals('run_gru_path_predictor', 'True')
    )

    #
    # trajectory_generation
    #
    run_mpc_traj_generation = LaunchConfiguration('run_mpc_traj_generation')
    run_mpc_traj_generation_arg = DeclareLaunchArgument(
        'run_mpc_traj_generation',
        default_value=os.environ.get('RUN_MPC_TRAJ_GENERATION', 'False'),
    )
    mpc_traj_generation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('trajectory_generation'),
                'launch/mpc_12state.launch.py'
            ])
        ]),
        launch_arguments={
            'yaml_path': os.environ.get('MPC_TRAJ_GEN_YAML_FILE', ''),
            'mpc_odom_topic': '/mavros/local_position/odom',
            'mpc_imu_topic': '/mavros/imu/data',
            'mpc_ref_path_topic': '/gru_predicted_path',
            'mpc_cmd_topic_arg': '/geometric_controller/multi_dof_setpoint'
        }.items(),
        condition=LaunchConfigurationEquals('run_mpc_traj_generation', 'True')
    )

    #
    # geometric controller node
    #
    run_geometric_controller = LaunchConfiguration('run_geometric_controller')
    run_geometric_controller_arg = DeclareLaunchArgument(
        'run_geometric_controller',
        default_value=os.environ.get('RUN_GEOMETRIC_CONTROLLER', 'False'),
    )
    geometric_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mav_controllers_ros'),
                'launch/geometric_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'yaml_path': os.environ.get('GEOMETRIC_CONTROLLER_YAML_FILE', '')
            }.items(),
        condition=LaunchConfigurationEquals('run_geometric_controller', 'True')
    )

    #
    # Geometric controller mavros interface node
    #
    geometric_to_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mav_controllers_ros'),
                'launch/geometric_to_mavros.launch.py'
            ])
        ]),
        launch_arguments={
            'param_file': os.environ.get('GEOMETRIC_MAVROS_YAML_FILE', ''),
            }.items(),
        condition=LaunchConfigurationEquals('run_geometric_controller', 'True')
    )


    ld.add_action(run_rs_launch_arg)
    ld.add_action(realsense_launch)

    ld.add_action(vins_config_launch_arg)
    ld.add_action(run_vins_launch_arg)
    ld.add_action(vins_launch)

    # ld.add_action(run_xrce_launch_arg)
    # ld.add_action(xrce_agent_launch)

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

    ld.add_action(run_apriltag_launch_arg)
    ld.add_action(apriltag_launch)
    ld.add_action(apriltag_tools_launch)

    ld.add_action(run_gru_launch_arg)
    ld.add_action(drone_path_predictor_launch)

    ld.add_action(run_mpc_traj_generation_arg)
    ld.add_action(mpc_traj_generation_launch)

    ld.add_action(run_geometric_controller_arg)
    ld.add_action(geometric_controller_launch)
    ld.add_action(geometric_to_mavros_launch)
    
    return ld
