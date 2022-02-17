
## ROS/ROS2

For users interested in using our [ROS](https://github.com/ifm/ifm3d-ros)
bindings, `ifm3d` and `ifm3d-ros` are both available in the ROS distribution
for Kinetic and Melodic (Noetic coming shortly).
```
$ sudo apt install ros-kinetic-ifm3d
```
or
```
$ sudo apt install ros-melodic-ifm3d
```

For users interested in using our [ROS2](https://github.com/ifm/ifm3d-ros2)
bindings, binaries will be included (starting with dashing) very soon. For now,
packages must be built from source. Do not use the debian mirror for
`ifm3d` since (depending on version) ROS2 ships parallel versions of some core
libraries (OpenCV, PCL) as compared with standard Ubuntu. `ifm3d` must be built
against the proper dependencies.