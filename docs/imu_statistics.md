# Computing IMU statistics for VIO systems
To use visual inertial odometry (VIO), or visual inertial SLAM (VI-SLAM), systems, IMU statistics need to be obtained.
Example of VIO: [open_vins](https://github.com/rpng/open_vins) which is used in the d2dtracker system.

The following section shows instrucitons on how to do obtain IMU statistics using `allan_ros2` package. This is only used in ROS2.

# allan_ros2
The original repo acn be found [here](https://github.com/CruxDevStuff/allan_ros2) which works  on ROS2 Foxy, and not humble.
To use it with ROS2 humble, you can use [this fork](https://github.com/nikola-j/allan_ros2/tree/humble).

Here, we assume using ROS2 humble.

You will need
* Clone `allan_ros2` into your ros2 workspace
```bash
# create a new ros2 workspace 
mkdir ~/allan_ws && cd ~/allan_ws
mkdir src && cd src

 # clone 'px4_msgs' 
git clone https://github.com/PX4/px4_msgs.git

git clone https://github.com/nikola-j/allan_ros2.git
cd allan_ros2 && git checkout humble

# install dependencies
cd ~/allan_ws
rosdep install --from-paths src -y --ignore-src

# build the workspace
colcon build 
```
# Configure
Populate the parameters in ```~/allan_ws/src/allan_ros2/config/config.yaml``` with the appropriate values for you bag. [example configuration files](https://github.com/CruxDevStuff/allan_ros2_dev/tree/main/config)
```yaml
allan_node:
  ros__parameters:
     topic: # /your/topic 
     bag_path: # path/to/bag.db3
     msg_type: # use 'ros' for sensor_msgs::msg::Imu, use 'px4' for px4_msgs::msg::VehicleImuStatus
     publish_rate: # your imu publish rate (hz) 
     sample_rate: # rate to sample data from bag. Higher sample rates take longer to compute 
```
Note : Requires rebuilding after configuration ```colcon build --packages-select allan_ros2```. This will be fixed in a future release.

# Run 
Launch the node 
```bash
   # requires building after changing config
   colcon build --packages-select allan_ros2
   
   # source the workspace 
   source ~/allan_ws/install/setup.bash
   
   # launch the node 
   ros2 launch allan_ros2 allan_node.py
```
# Output 
The node outputs ```deviation.csv``` and contains the computed raw deviation values for all 6 axis. 

### Launch output
```
[allan_node-1] [INFO] [1673168790.110605813] [allan_node]: IMU Topic set to : # topic in config.yaml
[allan_node-1] [INFO] [1673168790.110669795] [allan_node]: Sample rate set to : # sample rate in config.yaml
[allan_node-1] [INFO] [1673168790.111149862] [rosbag2_storage]: Opened database # path set in config.yaml
[allan_node-1] [INFO] [1673168793.933100186] [allan_node]: Bag length (Seconds): # detected bag length 
[allan_node-1] [INFO] [1673168793.933151985] [allan_node]: Sampling data from bag...
[allan_node-1] [INFO] [1673168799.354297658] [allan_node]: Total samples : # samples taken from bag 
[allan_node-1] [INFO] [1673168799.354340691] [allan_node]: Computing variance and deviation
[allan_node-1] [INFO] [1673168804.424160542] [allan_node]: DONE, deviation output logged to 'deviation.csv'
```

### Noise parameters
To obtain the noise paramaters run ```analysis.py``` with deviation data. 
```analysis.py``` outputs ```imu.yaml``` and deviation plots. ```imu.yaml``` contains the ```random_walk``` and ```noise_density``` values for the IMU and follows the kalibr [IMU Noise Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model) 
```bash
python3 src/allan_ros2/scripts/analysis.py --data deviation.csv
```
### Example ```imu.yaml```
```yaml
#Accelerometers
accelerometer_noise_density: 1.86e-03   # noise density (continuous-time)
accelerometer_random_walk:   4.33e-04   # bias random walk

#Gyroscopes
gyroscope_noise_density:     1.87e-04   # noise density (continuous-time)
gyroscope_random_walk:       2.66e-05   # bias random walk

rostopic:                               # fill your imu topic
update_rate:                            # fill your imu publish rate (hz)
```

You can use this `imu.yaml` directly in Kalibr when calibrating your camera-IMU system.