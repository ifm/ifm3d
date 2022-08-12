# How to: configure the camera

The O3R has multiple parameters that have an influence on the point cloud. Some of them affect the raw measurement and others modify how the data is converted into x,y,z, etc values. These parameters can be changed to better fit your applications and we are going to see how here. You can refer to [this page](documentation/O3R/Parameters/parameters:Settings%20Description) for a detailed description of each parameter.

There are multiple functions available to read the current configuration of the device and to set a new one. We are using JSON formatting.

For this process, we have to initialize the camera object (please have a look at the code example provided for full details of the imported libraries).
:::::{tabs}
::::{group-tab} Python
:::python
o3r = O3R()
:::
::::
::::{group-tab} C++
:::cpp
auto cam = std::make_shared<ifm3d::O3R>();
:::
::::
:::::

Note: if you are using multiple ifm devices (O3D, O3X, O3R), you can use the `Device` class.
:::::{tabs}
::::{group-tab} Python
:::python
cam = Device()
:::
::::
::::{group-tab} C++

If you need to use Device specific functions at a later point you can cast the pointer to the relevant class:

:::cpp
auto cam = ifm3d::Device::MakeShared();
auto cam_O3R = std::static_pointer_cast<ifm3d::O3R>(cam);
:::
::::
:::::

## Read the current configuration

The first provided function outputs the current configuration of the device (the VPU and each head currently attached). This function outputs the full configuration, including the parameters set for each camera head, but also other aspects like MAC and IP addresses, etc.
:::::{tabs}
::::{group-tab} Python
:::python
conf = cam.get();
:::
::::
::::{group-tab} C++
:::cpp
json conf = cam->Get();
:::
::::
:::::

## Write a new configuration

To write a new configuration to the device, you need to provide said configuration in json formatting. The provided configuration can be a subset or the full configuration.
:::::{tabs}
::::{group-tab} Python
:::python
o3r.set({'device':{'info':{'name':'great_o3r'}}})
:::
::::
::::{group-tab} C++
:::cpp
cam->Set(R"({"device":{"info": {"name": "my_o3r"}}})");
:::
::::
:::::

Note: we use [string literals](https://en.cppreference.com/w/cpp/language/string_literal) for easier readability.

To make the configuration persistent over reboots, you need to use the following function:
:::::{tabs}
::::{group-tab} Python
:::python
o3r.save_init()
:::
::::
::::{group-tab} C++
:::cpp
cam->SaveInit();
:::
::::
:::::

## The full example
:::::{tabs}
::::{group-tab} Python
:::{literalinclude} configuration.py
:language: python
:::
::::

::::{group-tab} C++
:::{literalinclude} configuration.cpp
:language: cpp
:::
::::
:::::