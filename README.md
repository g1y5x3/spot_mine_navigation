# Underground Mine Navigation using Spot

This project is developed to navigate Spot autonomously for miner safety monitoring using object detection with thermal images.

## Description


## Getting Started

### Dependencies

* [ros 2 humble](https://docs.ros.org/en/humble/index.html#)
* [spot_ros2](https://github.com/bdaiinstitute/spot_ros2)
* [velodyne](https://github.com/ros-drivers/velodyne/tree/humble-devel)

### Installing

In your ROS 2 workspace `src` directory, clone the repository:
```
git clone https://github.com/g1y5x3/spot_mine_navigation.git
cd <ros2_ws>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### Executing program

```
source install/setup.bash
ros2 run spot_mine_navigation thermal_publisher --ros-args --params-file src/spot_mine_navigation/config/spot_example.yaml
```
