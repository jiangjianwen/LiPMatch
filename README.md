## LiPMatch: LiDAR Point Cloud Plane based Loop-Closure


We present a point clouds based loop-closure method to correct long-term drift in Light Detection and Ranging based Simultaneous Localization and Mapping systems.

## Prerequisites

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **A-LOAM**
Follow [A-LOAM Installation](https://github.com/HKUST-Aerial-Robotics/A-LOAM).

### 1.2. **3DTK**
Follow [3DTK Installation](https://sourceforge.net/p/slam6d/discussion/939033/thread/1fc8f8f8/?limit=25).

### 1.3. **RangeNet(We are trying to add more segments to the graph!)**
Follow [RangeNet Installation](https://github.com/PRBonn/rangenet_lib).

## 2. Build LiPMatch
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/jiangjianwen/LiPMatch.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## Publication

If you use our implementation in your academic work, please cite the corresponding paper:

Jianwen Jiang, Jikai Wang, Peng Wang, Peng Bao and Zonghai Chen. *LiPMatch: LiDAR Point Cloud Plane based Loop-Closure*.

The BibTeX entry for the paper is: 
    
	@inproceedings{ jianwen2020ral, 
			author = {Jianwen Jiang, Jikai Wang, Peng Wang, Peng Bao and Zonghai Chen},
			title  = {LiPMatch: LiDAR Point Cloud Plane based Loop-Closure},
			booktitle = {IEEE RA-L},
			year = {2020}  
	}

## License

Copyright 2020 Jianwen Jiang, University of Science and Technology of China.


This project is free software made available under the MIT License. For details see the LICENSE file.

