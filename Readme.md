# Elevation Mapping

## 1. Dependencies

ROS 2 is extremely poorly supported, and a bunch of dependency issues can occur including but not limited to the following errors, tested on Ubuntu 20.04 runing Ros2-foxy.

### 1.1 pcl\_ros

Install from Debian pkg:

```shell
sudo apt install ros-$ROS_DISTRO-perception-pcl
```

~~Build from source:~~

1.  ~~Download source code of Perception-pcl pkg on branch ros2 from~~
    
2.  ~~Create a new ws and build.~~
    

~~**Errors**~~~~:~~

1.  ~~In~~ ~~_pointcloud\_to\_pcd.cpp_~~~~: line 56, change~~ ~~_#include <tf2\_eigen/tf2\_eigen.hpp>_~~ ~~to~~ ~~_#include <tf2\_eigen/tf2\_eigen.h>._~~ ~~Apply to all the tf2-related dependency-not-found issues.~~
    

### 1.2 Grid\_map

[grid\_map\_foxy.zip](https://alidocs.dingtalk.com/i/nodes/NZQYprEoWo4yGr9EFpBjp1K5V1waOeDk?iframeQuery=anchorId%253DX02lxtt0ihtrb3iy97u3u)

This package is from foxy-devel branch with grid\_map\_core pkg migrated from the humble branch. Neither of the original branches can pass compilation on my Ubuntu 20.04 with ROS2 Foxy.

**Errors**

1.  Filters/Filters.hpp not found.

```shell
sudo apt install ros-$ROS_DISTRO-filters
```

## 2. Build package

[https://github.com/GoAMATEUR/ElevationMapping-ROS2](https://github.com/GoAMATEUR/ElevationMapping-ROS2)

Adapted from ROS1 ver. of [https://github.com/ANYbotics/elevation\_mapping](https://github.com/ANYbotics/elevation_mapping)


```shell
cd ~/ros_ws/src
git clone https://github.com/GoAMATEUR/ElevationMapping-ROS2
cd ~/ros_ws && colcon build
```

## 3 Usage

### 3.1 Configuration

There are 4 config files inside of ./config folder.

*   elevation\_map.param.yaml: Define names of frames, elevation mapping behaviors, sensor models, etc.
    
*   topic\_name.yaml: Subscibed / publish topic names.
    
*   visualization.yaml
    
*   post\_processing.param.yaml
    

### 3.2 Subscribed Topics / TF's

**Point cloud** data of type PointCloud2 should be published to topic specified in _topic\_type.yaml_. 

**Estimate of Robot pose** w.r.t. global map is required by the node and should be published to specified topic of type PoseWithVarianceStamped. 

**TF** The following TF's should also be published.

*   map (global) -> robot\_frame
    
*   robot\_frame -> sensor\_frame.
    

### 3.3 Run Node

After bringing up sensors and odometry nodes, run: 

```shell
ros2 launch elevation_mapping_ros2 elevation_mapping.launch.py
```

To visualize the elevation map use rviz2 and subscirbe to PointCloud2 topics.

## References

[https://github.com/ANYbotics/elevation\_mapping](https://github.com/ANYbotics/elevation_mapping) ANYbotics

[https://github.com/leggedrobotics/elevation\_mapping\_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy/tree/main) cupy

elevation map to costmap

[https://github.com/ANYbotics/grid\_map](https://github.com/ANYbotics/grid_map)

<!-- [Elevation Mapping for Locomotion and Navigation using GPU](https://arxiv.org/abs/2204.12876)

*   2.5D map assigns height information for every grid cell. Memory saving and efficient. But need to handle hanging-over objects.
    
*   Sys overview:
    

Input Pose est (SLAM, ODOM, etc.) & Point cloud -> 

*   Height Cell update: Kalman Filter. Track an estimator h and variance. fuse observation and current estimate. Sensor noise model as a quadratic function of distance of point. 
    

    Outlier removal: Mahalanobis Distance + exclusion area.

    Vertical walls may appear shorter if naively using KF update 

[MEM: Multi-Modal Elevation Mapping for Robotics and Learning](https://arxiv.org/abs/2309.16818v1) -->