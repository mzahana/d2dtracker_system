#!/bin/bash -e

# This script clones the packages that are directly related to the d2dtracker system
# !!! WARNING !!! Other non-direct dependencies (e.g. mavros...etc) are not cloned/installed here. They are installed suring the setup of the d2dtracker container.

if [ -z "${DEV_DIR}" ]; then
  echo "Error: DEV_DIR environment variable is not set. Set it using export DEV_DIR=<DEV_DIR_deirectory_that_should_contain_PX4-Autopilot_and_ros2_ws>"
  exit 1
fi
echo "DEV_DIR=$DEV_DIR"
sleep 1
echo "GIT_USER=$GIT_USER"
echo "GIT_TOKEN=$GIT_TOKEN"
sleep 1

ROS2_WS=$DEV_DIR/ros2_ws
ROS2_SRC=$DEV_DIR/ros2_ws/src
PX4_DIR=$DEV_DIR/PX4-Autopilot
OSQP_SRC=$DEV_DIR


if [ ! -d "$ROS2_WS" ]; then
  echo "Creating $ROS2_SRC"
  mkdir -p $ROS2_SRC
fi


#
# Clone realsense-ros 4.51.1
#
# if [ ! -d "$ROS2_SRC/realsense-ros" ]; then
#     cd $ROS2_SRC
#     git clone https://github.com/IntelRealSense/realsense-ros.git -b 4.51.1
# else
#     cd $ROS2_SRC/realsense-ros
#     git checkout 4.51.1
# fi

PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/d2dtracker_system.git
else
    PKG_URL=https://github.com/mzahana/d2dtracker_system.git
fi

# Clone the d2dtracker_sim if it doesn't exist
if [ ! -d "$ROS2_SRC/d2dtracker_system" ]; then
    cd $ROS2_SRC
    git clone $PKG_URL
else
    cd $ROS2_SRC/d2dtracker_system && git pull origin main
fi

# Clone some PX4 rose-related packages
if [ ! -d "$ROS2_SRC/px4_msgs" ]; then
    cd $ROS2_SRC
    git clone https://github.com/PX4/px4_msgs.git
else
    cd $ROS2_SRC/px4_msgs && git pull origin main
fi

#
# custom px4_ros_com
#
PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    echo "GIT_USER=$GIT_USER , GIT_TOKEN=$GIT_TOKEN"
    echo
    PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/px4_ros_com.git
else
    echo "GIT_USER and GIT_TOKEN are not set"
    PKG_URL=https://github.com/mzahana/px4_ros_com.git
fi

if [ ! -d "$ROS2_SRC/px4_ros_com" ]; then
    cd $ROS2_SRC
    git clone ${PKG_URL}
else
    cd $ROS2_SRC/px4_ros_com && git checkout main && git pull origin main
fi



#
# d2dtracker_drone_detector
#
PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/d2dtracker_drone_detector.git
else
    PKG_URL=https://github.com/mzahana/d2dtracker_drone_detector.git
fi

if [ ! -d "$ROS2_SRC/d2dtracker_drone_detector" ]; then
    cd $ROS2_SRC
    git clone ${PKG_URL}
else
    cd $ROS2_SRC/d2dtracker_drone_detector && git checkout main && git pull origin main
fi


#
# multi_target_kf
#
PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/multi_target_kf.git
else
    PKG_URL=https://github.com/mzahana/multi_target_kf.git
fi

if [ ! -d "$ROS2_SRC/multi_target_kf" ]; then
    cd $ROS2_SRC
    git clone ${PKG_URL} -b ros2_humble
else
    cd $ROS2_SRC/multi_target_kf && git checkout ros2_humble && git pull origin ros2_humble
fi

#
# custom_trajectory_msgs
#
PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/custom_trajectory_msgs.git
else
    PKG_URL=https://github.com/mzahana/custom_trajectory_msgs.git
fi

if [ ! -d "$ROS2_SRC/custom_trajectory_msgs" ]; then
    cd $ROS2_SRC
    git clone ${PKG_URL} -b ros2_humble
else
    cd $ROS2_SRC/custom_trajectory_msgs && git checkout ros2_humble && git pull origin ros2_humble
fi

#
# trajectory_prediction (!! TODO !!! Should be removed. Use drone_path_predictor_ros instead)
#
PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    echo "GIT_USER=$GIT_USER , GIT_TOKEN=$GIT_TOKEN"
    echo
    PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/trajectory_prediction.git
else
    echo "GIT_USER and GIT_TOKEN are not set"
    PKG_URL=https://github.com/mzahana/trajectory_prediction.git
fi

if [ ! -d "$ROS2_SRC/trajectory_prediction" ]; then
    cd $ROS2_SRC
    git clone ${PKG_URL} -b ros2_humble
else
    cd $ROS2_SRC/trajectory_prediction && git checkout ros2_humble && git pull origin ros2_humble
fi
cd $ROS2_SRC/trajectory_prediction && . setup.sh

#
# drone_path_predictor_ros
#
PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    echo "GIT_USER=$GIT_USER , GIT_TOKEN=$GIT_TOKEN"
    echo
    PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/drone_path_predictor_ros.git
else
    echo "GIT_USER and GIT_TOKEN are not set"
    PKG_URL=https://github.com/mzahana/drone_path_predictor_ros.git
fi

if [ ! -d "$ROS2_SRC/drone_path_predictor_ros" ]; then
    cd $ROS2_SRC
    git clone ${PKG_URL}
    cd $ROS2_SRC/drone_path_predictor_ros && git checkout ros2_humble
else
    cd $ROS2_SRC/drone_path_predictor_ros && git checkout ros2_humble && git pull origin ros2_humble
fi

#
# trajectory_generation
#
PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    echo "GIT_USER=$GIT_USER , GIT_TOKEN=$GIT_TOKEN"
    echo
    PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/trajectory_generation.git
else
    echo "GIT_USER and GIT_TOKEN are not set"
    PKG_URL=https://github.com/mzahana/trajectory_generation.git
fi

if [ ! -d "$ROS2_SRC/trajectory_generation" ]; then
    cd $ROS2_SRC
    git clone ${PKG_URL}
    cd $ROS2_SRC/trajectory_generation && git checkout ros2_humble
else
    cd $ROS2_SRC/trajectory_generation && git checkout ros2_humble && git pull origin ros2_humble
fi
cd $ROS2_SRC/trajectory_generation && . setup.sh

#
# custom mav_controllers_ros
#
PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    echo "GIT_USER=$GIT_USER , GIT_TOKEN=$GIT_TOKEN"
    echo
    PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/mav_controllers_ros.git
else
    echo "GIT_USER and GIT_TOKEN are not set"
    PKG_URL=https://github.com/mzahana/mav_controllers_ros.git
fi

if [ ! -d "$ROS2_SRC/mav_controllers_ros" ]; then
    cd $ROS2_SRC
    git clone ${PKG_URL}
    cd $ROS2_SRC/mav_controllers_ros && git checkout ros2_humble
else
    cd $ROS2_SRC/mav_controllers_ros && git checkout ros2_humble && git pull origin ros2_humble
fi

#
# yolov8
# compatible with Ultralytics version  8.0.162
#
if [ ! -d "$ROS2_SRC/yolov8_ros" ]; then
    cd $ROS2_SRC
    git clone https://github.com/mgonzs13/yolov8_ros.git
    cd cd $ROS2_SRC/yolov8_ros && git checkout 2.0.1
else
    cd $ROS2_SRC/yolov8_ros && git pull origin && git checkout 2.0.1
fi

# cd $ROS2_WS && rosdep init && rosdep install --from-paths src --ignore-src -r -y

#
# arducam_ros2
#
PKG_URL=''
if [[ -n "$GIT_USER" ]] && [[ -n "$GIT_TOKEN" ]]; then
    PKG_URL=https://$GIT_USER:$GIT_TOKEN@github.com/mzahana/arducam_ros2.git
else
    PKG_URL=https://github.com/mzahana/arducam_ros2.git
fi
if [ ! -d "$ROS2_SRC/arducam_ros2" ]; then
    cd $ROS2_SRC
    git clone ${PKG_URL}
else
    cd $ROS2_SRC/arducam_ros2 && git pull origin main
fi

cd $ROS2_WS && colcon build

cd $DEV_DIR
