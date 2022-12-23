# FAST_DLC_SLAM

## Front_end : fastlio2      Back_end : lio_sam

A Robust and Effective LiDAR-SLAM System with Learning-based Denoising and Loop Closure

## Summary of the merits of our works based on previous works:

1.FAST-LIO2: The tightly coupled lio slam system lacks global consistency because of its lack of front-end. Refer to the back-end part of lio_sam and connect to GTSAM for back-end optimization.



3.Added on the basis of FAST_LIO_SLAM: 1. Euclidean distance-based loop detection search based on Radius Search, which increases the robustness of loop closure search; 2. The optimization result of loop detection is updated to the current frame pose of FAST-LIO2, and Refactor ikdtree to update subma.

## The additional Contributions  

1. Use the externally connected PGO loopback detection module for back-end optimization. FAST_LIO_SAM transplants the back-end GTSAM optimization part of LIO-SAM to the code of FAST-LIO2, and the data transmission and processing link is clearer.

2. Add keyframe saving, and save maps and tracks through rosservice commands.

3.The back-end optimization in FAST_LIO_SLAM only uses the high-level GPS for constraints. The high-level GPS is generally noisy, so the XYZ three-dimensional postion of GPS is added to constrain the GPS a priori factor.

## Prerequisites

- Ubuntu 18.04 and ROS Melodic
- PCL >= 1.8 (default for Ubuntu 18.04)
- Eigen >= 3.3.4 (default for Ubuntu 18.04)
- GTSAM >= 4.0.0(tested on 4.0.0-alpha2)

## Build

```shell
cd YOUR_WORKSPACE/src
git clone https://github.com/kahowang/FAST_LIO_SAM.git
cd ..
catkin_make
```

## Quick test

### Loop clousre：

#### 1 .For indoor dataset 

dataset is from yanliang-wang 's [FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC)  ,[dataset](https://drive.google.com/file/d/1NGTN3aULoTMp3raF75LwMu-OUtzUx-zX/view?usp=sharing) which includes `/velodyne_points`(10Hz) and `/imu/data`(400Hz).

```shell
roslaunch fast_lio_sam mapping_velodyne16.launch
rosbag play  T3F2-2021-08-02-15-00-12.bag  
```

![indoor](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11indoor.gif)



#### 2 .For outdoor dataset

dataset is from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) **Walking dataset:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

```shell
roslaunch fast_lio_sam mapping_velodyne16_lio_sam_dataset.launch
rosbag  play  walking_dataset.bag
```

![outdoor_1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11outdoor_1.gif)

![outdoor_2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11outdoor_2.gif)


#### 3.save_map

Enter the following command into the terminal, the map file will be saved in the appropriate folder:

```shell
rosservice call /save_map "resolution: 0.0
destination: ''" 
success: True
```

#### 4.save_poes

Enter the following command into the terminal, the poes file will be saved in the corresponding folder:

```shell
rosservice call /save_pose "resolution: 0.0
destination: ''" 
success: False
```

evo 绘制轨迹

```shell
evo_traj kitti optimized_pose.txt without_optimized_pose.txt -p
```

| ![evo1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11evo1.png) | ![evo2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11evo2.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

#### 5.some config 

```shell
# Loop closure
loopClosureEnableFlag: true		      # use loopclousre or not 
loopClosureFrequency: 4.0                     # Hz, regulate loop closure constraint add frequency
surroundingKeyframeSize: 50                   # submap size (when loop closure enabled)
historyKeyframeSearchRadius: 1.5             # meters, key frame that is within n meters from current pose will be considerd for loop closure
historyKeyframeSearchTimeDiff: 30.0           # seconds, key frame that is n seconds older will be considered for loop closure
historyKeyframeSearchNum: 20                  # number of hostory key frames will be fused into a submap for loop closure
historyKeyframeFitnessScore: 0.3              # icp threshold, the smaller the better alignment

# visual iktree_map  
visulize_IkdtreeMap: true

# visual iktree_map  
recontructKdTree: true

savePCDDirectory: "/fast_lio_sam_ws/src/FAST_LIO_SAM/PCD/"        # in your home folder, starts and ends with "/". Warning: the code deletes "LOAM" folder then recreates it. See "mapOptimization" for implementation
```



### Use GPS：

#### 1.dataset

dataset is from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) **Park dataset:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

```shell
roslaunch fast_lio_sam mapping_velodyne16_lio_sam_dataset.launch
rosbag  play  parking_dataset.bag
```

Line Color define:  path_no_optimized(blue)、path_updated(red)、path_gnss(green)

![gps_optimized_path](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11gps_optimized_path.gif)

![gps_optimized_with_map](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11gps_optimized_with_map.gif)

#### 2.save_map

输入如下指令到terminal中，地图文件将会保存在应文件夹中

```shell
rosservice call /save_map "resolution: 0.0
destination: ''" 
success: True
```

FAST-LIO  Map (no gnss prior factor)  Red   ;    FAST-LIO-SAM  (with gnss prior factor) Blue

![gps_map](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11gps_map.gif)


#### 3.save_poes

Obtain the updated ENU and enter the following command into the terminal, the poes file will be saved in the corresponding folder:

```
rosservice call /save_pose "resolution: 0.0
destination: ''" 
success: False
```

evo plot trajectories:

```
evo_traj kitti gnss_pose.txt optimized_pose.txt  -p
```

| FAST-LIO  (no gnss prior factor)                             | FAST-LIO-SAM  (with gnss prior factor)                       |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![evo_no_optimized](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11evo_no_optimized.png) | ![evo_optimized](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11evo_optimized.png) |



#### 4.some config 

```shell
# GPS Settings
useImuHeadingInitialization: false           # if using GPS data, set to "true"
useGpsElevation: false                      # if GPS elevation is bad, set to "false"
gpsCovThreshold: 2.0                        # m^2, threshold for using GPS data
poseCovThreshold: 0 #25.0                      # m^2, threshold for using GPS data  Pose Covariance Threshold: from isam2
```

#### 5.some fun

when you want to see the path in the Map [satellite map](http://dict.youdao.com/w/satellite map/#keyfrom=E2Ctranslation)，you can also use [Mapviz](http://wiki.ros.org/mapviz)p  plugin . You can refer to  my [blog](https://blog.csdn.net/weixin_41281151/article/details/120630786?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165569598716782246421813%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165569598716782246421813&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-120630786-null-null.142^v17^pc_search_result_control_group,157^v15^new_3&utm_term=MAPVIZ&spm=1018.2226.3001.4187) on CSDN.

![mapviz_1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11mapviz_1.gif)

![mapviz_2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11mapviz_2.gif)


## Attention:

1.In FAST-LIO2, the pose attitude is represented by so3, while in gtsam, the input relative_pose attitude is expressed in Euler RPY form, which needs to be converted and updated using Rodriguez's formula.

2. iktree  reconstruct 

3.In the walking data set, because some data are constantly holding the rotating lidar at the same place, the angle of the rotating lidar reaches the threshold for saving key frames. In a short period of time, multiple similar key frames are saved, causing ISAM2 to appear. If the feature degrades and the odometer runs away, the threshold parameters for key frame selection can be adjusted appropriately according to the data set.

4.Add the diamante part of the GPS prior prior factor, refer to the prior factor part of lio_sam, compared withFAST-LIO-SLAM, FAST-LIO-SLAM only uses the high-level constraints of GPS, and does not use constraints in the xy direction. However, the error of GPS in the high layer (Z axis) is relatively large, and it is easy to introduce errors in the optimization process.

5.Among the GPS prior factors, **"useGpsElevation"** selects the high-level constraint of GPS, which is not used by default, because the high-level noise of GPS is relatively large.

6.LIO-SAM uses **ekf_localization_node** this ROS Package to transfer the GPS WGS84 coordinate system to the World system. FAST-LIO-SAM considers decoupling with the external ROS package as much as possible, and calls **GeographicLib** for coordinate conversion.


