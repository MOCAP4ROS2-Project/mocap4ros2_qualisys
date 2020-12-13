# mocap4ros_qualisys

Requisites: qualisys_cpp_skd

- Just clone recurseivelly this repo to get https://github.com/qualisys/qualisys_cpp_sdk, or set QualisysSDK_PATH
- If you build qualisys_cpp_sdk in your workspace, the first time you build the workspace, exclude qualisys_driver:
```
colcon build --symlink-install --packages-skip qualisys_driver
colcon build --symlink-install
```
