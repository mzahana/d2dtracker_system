import launch
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file which brings up RealSense camera."""
    realsense_camera_node = Node(
        name='camera',
        namespace='camera',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
                'align_depth.enable': True,
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': True,
                'enable_depth': False,
                'depth_module.emitter_enabled': 0,
                'rgb_camera.profile': '848x480x30', 
                'depth_module.profile': '848x480x60',
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 200,
                'unite_imu_method': 2
        }]
    )

 
    return launch.LaunchDescription([ realsense_camera_node])