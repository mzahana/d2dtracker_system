#!/bin/bash -e

# This script sets up isacc_ros_* packages to run visual slam

if [ "${SETUP_ISAAC_ROS}" != "True" ]; then
    echo "SETUP_ISAAC_ROS variable is not set to True. Exiting script."
    exit 1
fi

if [ -z "${DEV_DIR}" ]; then
  echo "Error: DEV_DIR environment variable is not set. Set it using export DEV_DIR=<DEV_DIR_deirectory_that_should_contain_PX4-Autopilot_and_ros2_ws>"
  exit 1
fi

ROS2_WS=$DEV_DIR/ros2_ws
ROS2_SRC=$DEV_DIR/ros2_ws/src

if [ ! -d "$ROS2_WS" ]; then
  echo "Creating $ROS2_SRC"
  mkdir -p $ROS2_SRC
fi

# Initialize LFS
git lfs install --skip-repo

#
# Clone isaac_ros_common
#
if [ ! -d "$ROS2_SRC/isaac_ros_common" ]; then
    cd $ROS2_SRC
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
else
    cd $ROS2_SRC/isaac_ros_common
    git pull origin main
fi

#
# Clone isaac_ros_nitros
#
if [ ! -d "$ROS2_SRC/isaac_ros_nitros" ]; then
    cd $ROS2_SRC
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
else
    cd $ROS2_SRC/isaac_ros_nitros
    git pull origin main
fi

#
# Clone isaac_ros_visual_slam
#
if [ ! -d "$ROS2_SRC/isaac_ros_visual_slam" ]; then
    cd $ROS2_SRC
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
else
    cd $ROS2_SRC/isaac_ros_visual_slam
    git pull origin main
fi
