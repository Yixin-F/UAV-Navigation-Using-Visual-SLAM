## UAV Navigation System Using Visual SLAM in Dynamic Environment
### This repo has supported Pictures Dataset and ROS Bag, so it's simple enough to be built.
### RealsenseD435i and UAV

<img src="image/realsense.png" width="30%"><img src="image/uav.jpg" width="30%">

- Dynamic Object Detect (YOLOv8)
- Dense PointCloud Mapping
- Octomap Path Planning
- UAV Control Using PX4 (Thanks to Lishen Pu from Tongji)

### 0. Prerequisites
We have tested on:
>
> [OS] = Ubuntu 18.04
>
> [OpenCV] = 3.2
>
> [Eigen3]= 3.3.1
>
> [Pangolin] = 0.5
>
> [ROS] = Melodic
> 
> [libtorch] from [Baidu Netdisk](https://pan.baidu.com/s/1DQGM3rt3KTPWtpRK0lu8Fg?pwd=8y4k)
code: 8y4k

### 1. Build
```bash
cd ORB_SLAM3_Icarus
chmod +x build.sh
./build.sh
```

rgbd_tum, RGBD(ROS) and ros_RealsenseD435i (if you have your own equipments) targets will be build.

The frequency of camera topic must be lower than 15 Hz.

You can run this command to change the frequency of topic which published by the camera driver in /launch file, but it's optional
```bash
roslaunch camera_topic_remap.launch
```

### 2. SLAM
#### TUM Dataset
From https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download

<centre>
<img src="image/office.gif" width="50%">
</centre>

```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
```
#### Evaluation on TUM
| Dynamic ORB SLAM2 | ORB SLAM2
:----:|:-------:
![ate Dynamic](image/ate_Dynamic.png) | ![ate ORB](image/ate_ORB.png)

#### Your own datasets in ROS
Remember to change the topic in ros_rgbd.cc  !!

##### OpenLORIS-Scene Dataset
From https://lifelong-robotic-vision.github.io/dataset/scene

```bash
roscore
./Examples/ROS/RGBD Vocabulary/ORBvoc.txt  PATH_TO_SETTINGS_FILE
rosbag play YOUR_OWN_ROS_BAG 
```
<centre>
<img src="image/market.gif" width="50%">
</centre>

### 3. Path Planning
#### Get an Otcomap in .bt (/map files)
```bash
roslaunch rrtInteractive.launch (cd to this file)
./Examples/Planner/path_planner 
```
#### Interact with 3D Navigation button
Select your start and goal points in rviz.

<centre>
<img src="image/rrt.png" width="40%">
</centre>

### 4. UAV Control (code is comming soon...)
#### 1) Preparation
Make sure that your gazebo/mavlink is working !!

You should learn how to establish your own project with [PX4](https://px4.io/) or [XTDrone](https://www.yuque.com/xtdrone/manual_cn)

#### 2) Gazebo Simulation and QGround
<centre>
<img src="image/gazebo.png" width="50%">
</centre>

#### 3) Octomap Navigation and PX4 Control
<centre>
<img src="image/navigation.gif" width="50%">
</centre>
<centre>
<img src="image/px4.gif" width="50%">
<centre>

