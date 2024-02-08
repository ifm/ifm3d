# How to: configure the camera

The O3R has multiple parameters that have an influence on the point cloud. Some of them affect the raw measurement and others modify how the data is converted into x,y,z, etc values. These parameters can be changed to better fit your applications and this document presents how. You can refer to [this page](https://ifm3d.com/latest/Technology/3D/index_3d.html) for a detailed description of each parameter.

The ifm3d API provides functions to read and set the configuration of the device. Note that JSON formatting is used for all the configurations.

For this process, an O3R object has to be initialized to establish a connection with the device (please have a look at the code example provided for full details of the imported libraries).
:::::{tabs}
::::{group-tab} Python
:::python
o3r = O3R()
:::
::::
::::{group-tab} C++
:::cpp
auto o3r = std::make_shared<ifm3d::O3R>();
:::
::::
:::::

Note: if you are using multiple ifm devices (O3D, O3X, O3R), you can use the `Device` class.
:::::{tabs}
::::{group-tab} Python
:::python
dev = Device()
:::
::::
::::{group-tab} C++

If you need to use functions specific to the concrete `Device` subclass, you can cast the pointer to the relevant class:

:::cpp
auto dev = ifm3d::Device::MakeShared();
// Only do this if you're sure the `Device` is an instance of `O3R`:
auto dev_O3R = std::static_pointer_cast<ifm3d::O3R>(dev);
auto init_status = dev_O3R->GetInitStatus();


// Otherwise use dynamic_pointer_cast and check the value for nullptr:
auto dev_O3R = std::dynamic_pointer_cast<ifm3d::O3R>(dev);
if (dev_O3R)
{
   auto ports = dev_O3R->Ports();
}
:::
::::
:::::

:::{note}
The `GetInitStatus` returns the status of the device's initialization process. Use it to ensure the device is properly booted up before querying for data.
:::

## Read the current configuration

The first provided function outputs the current configuration of the device (the VPU and each head currently attached). This function outputs the full configuration, including the parameters set for each camera head, but also other aspects like MAC and IP addresses, etc.
:::::{tabs}
::::{group-tab} Python
:::python
conf = o3r.get();
:::
::::
::::{group-tab} C++
:::cpp
json conf = o3r->Get();
:::
::::
:::::

## Write a new configuration

To set a new configuration, you need to provide said configuration in JSON formatting. The provided configuration can be a subset or the full configuration, as long as it follows the proper JSON hierarchy.

:::::{tabs}
::::{group-tab} Python
:::python
o3r.set({'device':{'info':{'name':'great_o3r'}}})
:::
::::
::::{group-tab} C++
:::cpp
o3r->Set(R"({"device":{"info": {"name": "my_o3r"}}})");
:::
::::
:::::

Note: we use [string literals](https://en.cppreference.com/w/cpp/language/string_literal) for easier readability.

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
