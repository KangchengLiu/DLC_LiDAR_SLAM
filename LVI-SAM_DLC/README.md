# NEW Feature
This repo is adapted based on the original LVI_SAM, which makes it easier to adapt your sensor.

1. The original version of lio_sam sets many sensor external parameters to fixed values in the code, this version extracts the external parameters into the yaml file, making it easier to configure;

2. The original version of lio_sam dose not consider the translation amount between lidar and camera, this version adds the translation amount;

3. The "imu2LidarExtrinsicRotation" 、“extrinsicRotation”、“extrinsicTranslation”、"lcExtrinsicRotation" and "lcExtrinsicTranslation" in "config/params_camera.yaml" needs to be set as your device extrinsic.


This repository contains code for a lidar-visual-inertial odometry and mapping system, which combines the advantages of [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/a246c960e3fca52b989abf888c8cf1fae25b7c25) and [Vins-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) at a system level.



## Dependency

- [ROS](http://wiki.ros.org/ROS/Installation) (Tested with kinetic and melodic)
- [gtsam](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)
  ```
  sudo add-apt-repository ppa:borglab/gtsam-release-4.0
  sudo apt install libgtsam-dev libgtsam-unstable-dev
  ```
- [Ceres](https://github.com/ceres-solver/ceres-solver/releases) (C++ library for modeling and solving large, complicated optimization problems)
  ```
  sudo apt-get install -y libgoogle-glog-dev
  sudo apt-get install -y libatlas-base-dev
  wget -O ~/Downloads/ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip
  cd ~/Downloads/ && unzip ceres.zip -d ~/Downloads/
  cd ~/Downloads/ceres-solver-1.14.0
  mkdir ceres-bin && cd ceres-bin
  cmake ..
  sudo make install -j4
  ```

### Getting start with Docker  

When you use Docker, you could solve the dependency at once.  
For more information, you can check [docker_start.md](./docker/docker_start.md).    

---

## Compile

You can use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone https://github.com/TixiaoShan/LVI-SAM.git
cd ..
catkin_make
```

---

## Datasets

The datasets used in the paper can be downloaded from Google Drive. The data-gathering sensor suite includes: Velodyne VLP-16 lidar, FLIR BFS-U3-04S2M-CS camera, MicroStrain 3DM-GX5-25 IMU, and Reach RS+ GPS.

```
https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV?usp=sharing
```

**Note** that the images in the provided bag files are in compressed format. So a decompression command is added at the last line of ```launch/module_sam.launch```. If your own bag records the raw image data, please comment this line out.

---

## Run the package

1. Configure parameters:

```
Configure sensor parameters in the .yaml files in the ```config``` folder.
```

2. Run the launch file:
```
roslaunch lvi_sam run.launch
```

3. Play existing bag files:
```
rosbag play handheld.bag 
```

---

## Related Packages

  - [LVI_SAM_fixed by epicjung](https://github.com/epicjung/LVI_SAM_fixed)
  - [LVI-SAM-modified by skyrim835](https://github.com/skyrim835/LVI-SAM-modified)

---

## TODO

  - [ ] Update graph optimization using all three factors in imuPreintegration.cpp, simplify mapOptimization.cpp, increase system stability 

---
