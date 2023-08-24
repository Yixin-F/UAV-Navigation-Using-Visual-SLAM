# ORB_SLAM3_Icarus (SEU)
# This repo has supported Pictures Dataset and ROS Bag, so it's simple enough to be built.
**This code is an extended version of [YOLO_ORB_SLAM3_with_pointcloud_map](https://github.com/YWL0720/YOLO_ORB_SLAM3_with_pointcloud_map), and several bugs have been fixed.**
- Dynamic Object Detect (YOLOv8)
- Dense PointCloudMapping
- Octomap path planning (Thanks to Shenpu Li in TJU, but comming soon)

### 0. Prerequisites
We have tested on:
>
> OS = Ubuntu 18.04
>
> OpenCV = 3.2
>
> [Eigen3]= 3.3.1
>
> [Pangolin] = 0.5
>
> [ROS] = Melodic
> [libtorch] from [Baidu Netdisk](https://pan.baidu.com/s/1DQGM3rt3KTPWtpRK0lu8Fg?pwd=8y4k)
code: 8y4k

### 1. Build
```bash
cd ORB_SLAM3_Icarus
chmod +x build.sh
./build.sh
```

rgbd_tum,  RGBD(ROS) and ros_RealsenseD435i (if you have your own equipments) targets will be build.

The frequency of camera topic must be lower than 15 Hz.

You can run this command to change the frequency of topic which published by the camera driver in /launch file, but it's optional
```bash
roslaunch camera_topic_remap.launch
```

### 2. Try
#### TUM Dataset
From https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download

#### OpenLORIS-Scene Dataset
From https://lifelong-robotic-vision.github.io/dataset/scene

```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
```

#### Your own datasets in ROS
Remember to change the topic in ros_rgbd.cc  !!
```bash
roscore
./Examples/ROS/RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
rosbag play YOUR_OWN_ROS_BAG 
```
