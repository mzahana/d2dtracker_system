# Camera-IMU Calibration
VIO or VI-SLAM systems require cameras and IMUs to be calibrated. This is basicallau computing their intrinsics and extrinsics.
[Kalibr](https://github.com/ethz-asl/kalibr) is a powerful tool to do this task. The doument exokains this process.

# Calibration target
You need a claibration target to calibrate your cameras. Use an apriltag target on A0 paper. Informaion on calibration targets can be found [here](https://github.com/ethz-asl/kalibr/wiki/calibration-targets).

You also need to prepare a `yaml` file for the calibration target. It should like something like this.
`apriltag_6x6.yaml`
```bash
target_type: 'aprilgrid' #gridtype
tagCols: 6               #number of apriltags
tagRows: 6               #number of apriltags
tagSize: 0.088           #size of apriltag, edge to edge [m]
tagSpacing: 0.3          #ratio of space between tags to tagSize
                         #example: tagSize=2m, spacing=0.5m --> tagSpacing=0.25[-]
```

# Recording ROS bag
First you need to record a ros bag the contains your camera topics and IMU topics. For example, if you are using the Intel Realsense D455 or D435i cameras, you need to record the two infra image topics and the imu topic. Image topics frame rate can be around 30Hz. IMU rate should be around 200Hz or more.

If you recrod a ROS 2 bag, you will need to convert it to ROS 1 so Kalibr can process it.
You need the `rosbags` package.

Convert the ROS2 folder (contains the database and metadata file) into the ROS1 bag file. If you recorded other non-image IMU topics (or non-standard message types) use --exclude-topic to not convert them. 
```bash
pip3 install rosbags>=0.9.12 # might need -U
rosbags-convert <ros2_bag_folder> --dst calib_01.bag --exclude-topic <non_img_and_imu_topics>
```


**NOTES**
1. Avoid fast motion during recording your ros bag
2. Make sure the apriltags are visible in your cameras and cover your camera frames
3. Make sure to excite all IMU axes during your recording
4. Adequate ros bag duration is between 30-60 seconds

# Camera Calibration
Reference: see [here](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration).

Video: see [here](https://www.youtube.com/watch?v=BtzmsuJemgI&t=600s).

Once you have the ROS 1 bag, you can use Kalibr toolbox to calibrate your cameras and compute their intrinsics.

It is recommende to use the Kalibr Docker image to speed up this process and avoiding wasting time on the manual setup process.

Clone the Kalibr repository, and build the docker image
```bash
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
docker build -t kalibr -f Dockerfile_ros1_20_04 . # change this to whatever ubuntu version you want
```

We can now mount the data folder in the container /data path and enter the command prompt. 
```bash
FOLDER=/path/to/your/data/on/host
xhost +local:root
docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" kalibr
```

In the docker terminal, you can run the calibration command as follows.
```bash
rosrun kalibr kalibr_calibrate_cameras \
 	--target apriltag_6x6.yaml \
 	--models pinhole-radtan pinhole-radtan \
 	--topics /cam0/image_raw /cam1/image_raw \
 	--bag cam_april.bag \
 	--bag-freq 10.0 \
    --show-extraction
```

Inspect the output `.pdf` report and make sure you have valid calibration, see [here](https://github.com/ethz-asl/kalibr/wiki/multiple-camera-calibration#3-the-output).

# Camera-IMU calibration
Reference: See [here](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration).

Video: See [here](https://www.youtube.com/watch?v=BtzmsuJemgI&t=1653s).

This step requires the `imu.yaml` file that you obtain from the IMU [statistics step](imu_statistics.md), and the camchain yaml file `camchain-%BAGNAME%.yaml` obtained in the previous step (camera calibration).

**NOTE** It is also goof to inflate the IMU statistics in the `imu.yaml`. Multiply the white noises by 5, and the random walks by 10, before you use it in the next step.

Inside the Kalibr docker container, execute the calibration command as follows.
```bash
rosrun kalibr kalibr_calibrate_imu_camera \
	--target apriltag_6x6.yaml \
	--imu imu.yaml \
	--imu-models scale-misalignment \
    --reprojection-sigma 1.0 \
	--cams cam_april-camchain.yaml \
	--bag imu_april.bag \
    --show-extraction
```

Inspect the output `.pdf` report and make sure the calibration is valid. You should have constatnt sampling rate of your IMU (not like the one showed [here](https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration#4-the-output)).

See how to inspect the generated report in [this video](https://youtu.be/rBT5O5TEOV4?t=1874).

You should also get a yaml file `camchain-imucam-%BAGNAME%.yaml` which you are going to use in your VIO or VI-SLAM system along with the `imu.yaml` file.