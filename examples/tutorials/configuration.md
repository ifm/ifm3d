# How to: configure the camera

Corresponding code example: [configuration_ex.cpp](configuration_ex.cpp)

The O3R has multiple parameters that have an influence on the point cloud. Some of them affect the raw measurement and others modify how the data is converted into x,y,z, etc values. These parameters can be changed to better fit your applications and we are going to see how here. You can refer to [this page](INSERT-LINK) for a detailed description of each parameter.

At this stage, there are two functions available to read the current configuration of the device and to set a new one. We are using JSON formatting.

For this process, we have to initialize the camera object (please have a look at the code example provided for full details of the imported libraries).

```cpp
auto cam = ifm3d::Camera::MakeShared();
```

## Read the current configuration

The first provided function outputs the current configuration of the device (the VPU and each head currently attached). This function outputs the full configuration, including the parameters set for each camera head, but also other aspects like MAC and IP addresses, etc.
```cpp
json conf = cam->ToJSON();
```

## Write a new configuration 

To write a new configuration to the device, you need to provide said configuration in json formatting. 
```cpp
cam->FromJSON(conf);
```