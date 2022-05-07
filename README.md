# VI-LOAM_ISLAB

## Visual Inertial Lidar Odometry and Mapping

This repository contains code for a VI-LOAM system, which combines the advantages of [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM.git) and [Vins-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) at a system level. 

A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver to simplify code structure. This code is modified from LOAM and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED). This code is clean and simple without complicated mathematical derivation and redundant operations. 

<img src="https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/picture/kitti.png" width = 55% height = 55%/>

**Modifier:** [Didula Dissanayaka](https://dissanayakadidula.wixsite.com/diduladissanayaka)


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04, 18.04 or 20.04.
ROS Kinetic, Melodic or. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

### 1.4. **OpenCV 3.4 or higher**
Follow [OpenCV Installation](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html).


## 2. Build A-LOAM
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/didzdissanayaka8/VI-LOAM_ISLAB.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Datasets

## LVI-SAM Dataset


Download [LVI-SAM Dataset](https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV?usp=sharing). The dataset include following sensors: Velodyne VLP-16 lidar, FLIR BFS-U3-04S2M-CS camera, MicroStrain 3DM-GX5-25 IMU, and Reach RS+ GPS.

**Note** that the images in the provided bag files are in compressed format. So a decompression command is added at the last line of ```launch/module_sam.launch```. If your own bag records the raw image data, please comment this line out.

## AI4L Dataset

## 4. Run the package

1. Configure parameters:

```
Configure sensor parameters in the .yaml files in the ```config``` folder.
```

2. Run the launch file:
```
roslaunch viloam run.launch
```

3. Play existing bag files:
```
rosbag play handheld.bag 


## 5.Acknowledgements


## Acknowledgement

  - The visual-inertial odometry module is adapted from [Vins-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).
  - The lidar-inertial odometry module is adapted from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/a246c960e3fca52b989abf888c8cf1fae25b7c25).
  - Tightly-coupled lidar-visual inertial odometry and mapping [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM.git)
