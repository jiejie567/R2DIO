# R2DIO
## R2DIO: Robust, Real-Time and Depth-Inertial Odometry Based on Multi-Modal Constraints (Intel Realsense L515 as an example)

The RGB-D camera is widely applied on lightweight robots as an essential sensor in indoor SLAM. However, limited by computing cost, many RGB-D SLAM systems do not fully use the multi-modal information of cameras, resulting in degeneration and low accuracy in challenging scenes. To address this issue, this letter introduces a novel, lightweight, robust, real-time and depth-inertial odometry (R2DIO) for ToF RGB-D cameras. Texture and structure information is coupled simultaneously to improve robustness. It efficiently extracts line features from RGB images and plane features from depth images based on agglomerative clustering. When aligning features, the direction vectors of lines and planes are used to filter the mismatches and enhance real-time capability. The IMU measurements propagate states and are tightly coupled in the system by pre-integration. Finally, the system estimates states and builds dense, colored maps with the following constraints: line and plane matching constraints, IMU pre-integration constraints and historical odometry constraints. We demonstrate the robustness, accuracy and efficiency of R2DIO in the experiments. The competitive results indicate that our system is able to locate precisely in challenging scenes and run at 30 Hz on a low-power platform. The source code of R2DIO and experiment datasets is publicly available for the development of the community. 

A summary video demo can be found at [Youtube](https://www.youtube.com/watch?v=YqJZTE948sk) and [Bilibili](https://www.bilibili.com/video/BV1GW4y1u7sz/).

**Author:** Xu Jie, Harbin Institute of Technology, China

## 1. SLAM examples with Intel RealSense L515
### 1.1 In a hall
<p align='center'>
<a href="https://www.youtube.com/watch?v=YqJZTE948sk">
<img width="65%" src="/img/hall.png"/>
</a>
</p>

### 1.2 In an office
<p align='center'>
<a href="https://www.youtube.com/watch?v=YqJZTE948sk">
<img width="65%" src="/img/office.png"/>
</a>
</p>

### 1.3 Facing a display board
<p align='center'>
<a href="https://www.youtube.com/watch?v=YqJZTE948sk">
<img width="65%" src="/img/displayboard.png"/>
</a>
</p>

## 2. Prerequisites
### 2.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 20.04.

ROS noetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html). 

Tested with 2.0

### 2.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html). 

Tested with 1.10 (inherent in ros)

### 2.4. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt update
sudo apt install ros-noetic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

### 2.4. **OpenCV**
Tested with 4.2.0 (inherent in ros)

## 3. Build 
### 3.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/jiejie567/R2DIO.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

### 3.2 Download test rosbag
You may download our [recorded data](https://drive.google.com/drive/folders/1S0Q351M9zEgg-Nme4obqexRQlYRJyco7?usp=sharing) (10GB). (one rosbag)

If you are in China, you can download the recorded data via Baidu Netdisk
: [office](https://pan.baidu.com/s/1LTos6MG4CUq3SJz6GV55tQ), [hall](https://pan.baidu.com/s/16hp1APONPAn46WgFgvkm_g), and [display board](https://pan.baidu.com/s/1Ys_a9dZR9E-d9ELlY6-wug). The extraction code is 0503. (ten rosbags)

Note that due to the limitation of Google drive capacity, we only upload one rosbag. 




### 3.3 Launch ROS
```
    roslaunch r2dio r2dio.launch
```
Note that change the path to your datasets.

## 4. Sensor Setup
If you have new Realsense L515 sensor, you may follow the below setup instructions

### 4.1 IMU calibration (optional)
You may read official document [L515 Calibration Manual] (https://github.com/l515_calibration_manual.pdf) first

use the following command to calibrate imu, note that the build-in imu is a low-grade imu, to get better accurate, you may use your own imu
```
cd ~/catkin_ws/src/r2dio/l515_imu_calibration
python rs-imu-calibration.py
```

### 4.2 L515
<p align='center'>
<img width="35%" src="/img/setup.png"/>
</p>

### 4.3 Librealsense
Follow [Librealsense Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)

### 4.4 Realsense_ros
Copy [realsense_ros](https://github.com/IntelRealSense/realsense-ros) package to your catkin folder
```
    cd ~/catkin_ws/src
    git clone https://github.com/IntelRealSense/realsense-ros.git
    cd ..
    catkin_make
```

### 4.5 Launch ROS
Make Lidar still for 1 sec to estimate the initial bias, otherwise will cause localization failure!
```
    roslaunch r2dio r2dio_L515.launch
```
### 4.6 Docker suppot
You can see the details in docker_support.pdf. Thanks for [Yin Ao](https://github.com/coolaogege)'s work about the docker.
## 5. Citation
If you use this work for your research, you may want to cite the paper below, your citation will be appreciated.
```
@ARTICLE{10268066,
  author={Xu, Jie and Li, Ruifeng and Huang, Song and Zhao, Xiongwei and Qiu, Shuxin and Chen, Zhijun and Zhao, Lijun},
  journal={IEEE Transactions on Instrumentation and Measurement}, 
  title={R2DIO: A Robust and Real-Time Depth-Inertial Odometry Leveraging Multimodal Constraints for Challenging Environments}, 
  year={2023},
  volume={72},
  number={},
  pages={1-11},
  doi={10.1109/TIM.2023.3320753}}

@article{wang2021lightweight,
  author={H. {Wang} and C. {Wang} and L. {Xie}},
  journal={IEEE Robotics and Automation Letters}, 
  title={Lightweight 3-D Localization and Mapping for Solid-State LiDAR}, 
  year={2021},
  volume={6},
  number={2},
  pages={1801-1807},
  doi={10.1109/LRA.2021.3060392}}
```
## 6. Acknowledgements
The code is heavily derived from [SSL-SLAM3](https://github.com/wh200720041/ssl_slam3), thanks for Wang Han's open-source spirit.

The datasets and video are processed by [Qiu Shuxin](https://github.com/1136958879) and Huang Song.

The docker is created by [Yin Ao](https://github.com/coolaogege).

What's more, the "DIO" in the title "R2DIO" means:
<p align='center'>
<img width="35%" src="/img/dio.jpg"/>
</p>

