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
                'align_depth.enable': False,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_color': False,
                'enable_depth': False,
                'enable_gyro': False,
                'enable_accel': False,
                'gyro_fps': 200,
                'accel_fps': 200,
                'enable_pose': True,
                'unite_imu_method': 2
        }]
    )

 
    return launch.LaunchDescription([ realsense_camera_node])