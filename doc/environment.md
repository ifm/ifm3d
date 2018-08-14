# ifm3d - Environment Variables

This document will help you to understand the different environment variables
used with ifm3d library

Following is the list of environment variables used with ifm3d.

- [IFM3D_IP](#IFM3D_IP)
- [IFM3D_SESSION_ID](#IFM3D_SESSION_ID)

## IFM3D\_IP

IFM3D\_IP environment variable is used to set the device IP. If you fail to pass
device IP while creating ``ifm3d::Camera`` class object then ifm3d will look for
the environment variable IFM3D\_IP for the device IP value. If IFM3D\_IP is not
set in the environment variable then ifm3d will try to connect to default
IP(192.168.0.69)

Setting the variable on Linux

```console
$ export IFM3D_IP=192.168.0.68
```
Setting the value on Windows

```console
> set IFM3D_IP=192.168.0.68
```

## IFM3D\_SESSION\_ID

This is used to pass the user session ID to ifm3d. Typically the sensor
generates a unique Session ID which is used to prevent multiple control instance
from interfering each other. But during development sometimes it is intended to
reuse a session. This can be the case during debugging sessions when it gets
annoying to wait until the session automatically is closed. But listen to Uncle
Ben, with great power comes great responsibility.  Use this feature wisely. The
session value must be a 32 char long hex value.

Setting the variable on Linux

```console
$ export IFM3D_SESSION_ID=3c701f01bc3346c0940dac33f85ef8e6
```
Setting the value on Windows

```console
> set IFM3D_SESSION_ID=3c701f01bc3346c0940dac33f85ef8e6
```

