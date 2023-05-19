# mocap4ros_qualisys

[![GitHub Action Status](https://github.com/MOCAP4ROS2-Project/mocap4ros2_qualisys/actions/workflows/main.yaml/badge.svg)](https://github.com/MOCAP4ROS2-Project/qualisys_driver)

[![codecov](https://codecov.io/gh/MOCAP4ROS2-Project/mocap_optitrack_driver/main/graph/badge.svg)](https://codecov.io/gh/MOCAP4ROS2-Project/mocap_optitrack_driver)

Requisites: qualisys_cpp_skd

Create workspace:
```
mkdir -p mocap_ws/src && cd mocap_ws/src
```
Download qualisys repo recursively to get https://github.com/qualisys/qualisys_cpp_sdk, or set QualisysSDK_PATH:
```
git clone --recursive https://github.com/MOCAP4ROS2-Project/mocap4ros2_qualisys.git
```
Install dependencies:
```
vcs import < mocap4ros2_qualisys/dependency_repos.repos
```
Compiling workspace:
```
cd .. && colcon build --symlink-install
```
Source workspace:
```
source install/setup.bash
```
Setup your qualisys configuration:
```
mocap_ws/src/mocap4ros2_qualisys/qualisys_driver/config/qualisys_driver_params.yaml
```
Launch optitrack system:
```
ros2 launch qualisys_driver qualisys.launch.py
```
Visualize in rViz:
```
ros2 launch mocap_marker_viz mocap_marker_viz.launch.py mocap_system:=qualisys
```
