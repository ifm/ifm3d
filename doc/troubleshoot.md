ifm3d - Troubleshooting Guide
=============================

You can use this guide to help you identify and resolve basic problems you may
be experiencing with ifm3d library or ifm 3d cameras.

# Select the troubleshooting options

- [Connecting the camera](#Connecting-the-camera)
- [ifm3d logging](#ifm3d-logging)
- [ifm3d not found](#ifm3d-not-found)
- [WaitForFrame Timeout](#waitforframe-timeout)

## Connecting the camera
While connecting to device through ifm3d e.g.

```
    $ ifm3d ls
```

if you get following error message

```
ifm3d error: -100001
Lib: XMLRPC Timeout - can you `ping' the sensor?
```

This shows that ifm3d is not able to connect to device on your network.

Following are some guidelines to troubleshoot this issue

##### Provide the device IP address to the ifm3d

ifm3d by default connects to the default IP <"192.168.0.69">, one can also pass
the IP through enviornment variable IFM3D_IP and for command line tools through
`--ip` switch. To get the application list from camera one can use following
command

```
    $ ifm3d --ip=192.168.0.69 ls
    $ IFM3D_IP=192.168.0.69 ifm3d ls
```

##### Check if you can ping the camera

Power on the network and camera and check if you are able to ping the
camera. If not successfull check for the physical connections.

##### If you are behind the proxy network

Ping to device might not be successful if you are under proxy network, to
resolve this issue one needs to bypass the proxy for the device address. A
quick fix is to set the environment variable `no_proxy` to the IP address of
the device. If you are working with multiple devices it is always good to
bypass the required IP's in `~/.bashrc` file as shown below:

```
    $ vi ~/.bashrc
    no_proxy="127.0.0.1, localhost,  192.168.0.69, 192.168.0.70, 192.168.1.* "
```

##### DHCP enabled device on DHCP network

On Windows, use IVA
[ifm Vision Assistant](https://www.ifm.com/de/de/product/E3D300) software to
search the device on the network and obtain the IP Address of device

On Linux,use following command
```
 $ avahi-browse -arp 2>/dev/null | grep  ^=.*O3D.*Workstation
```

## ifm3d logging

This section will help you to understand the different log levels used in ifm3d

##### ifm3d log levels

ifm3d defines several severity levels in its logging subsystem. These are
defined [here](../modules/camera/src/libifm3d_camera/logging.cpp). The
primarily used log levels are `IFM3D_TRACE` and `IFM3D_PROTO_DEBUG`. The former
is used for logging general flow control of the software at a verbosity level
suitable for debugging general issues, while the latter helps debug issues
associated with the wire protocol between this library and the hardware.

##### Increasing the log levels at runtime

When running the `ifm3d` command-line program or your own program which linked
to an `ifm3d` shared library, the logging can be turned up to get more
info. For example:

```
    $ IFM3D_VLOG=15 ifm3d config < foo.json
```

The log output can be found at ``/tmp/ifm3d.INFO`` There is a file for each log
level `INFO`, `WARNING`, and `ERROR`.

##### Changing the log directory

You can also change the GLog log directory with following commad

```
    $ GLOG_log_dir=<user log dir path> ifm3d config < foo.json
```

For more help on GLOG visit
[Glog Document](https://godoc.org/github.com/golang/glog#pkg-files)

## ifm3d not found

This might occur if ifm3d is not installed properly.

##### Troubleshoot on Windows

Set the ifm3d executable path in the enviornment path of the system. This is
explained very well
[here](https://github.com/ifm/ifm3d/blob/master/doc/windows.md#running-ifm3d-tool-on-windows).

##### Troubleshoot on Linux

Use intruction at the https://github.com/ifm/ifm3d#the-default-build to
build and install the library on Linux.

## WaitForFrame Timeout

During runtime it can occur that the camera reports a Timeout when calling for
example ``fg->WaitForFrame(im.get(), 1000)``.
This can be caused by an issue on the camera possible root causes:

- Overheat, due to the active illumination and the configuration it can appear
  that the camera is getting too hot.
  So the camera needs to stop the image acquisition until it is cooled down.
- Issues with the power supply. The O3D3XX does have an internal power supply
  monitoring. If an under or over voltage  situation is detected the image
  acquisition is also stopped.
- Other none recoverable hardware issues detected by the on board camera
  diagnostics.

During development and for getting support on [GitHub](https://github.com/ifm/ifm3d/issues)
the ``ifm3d trace`` command is a powerful information source.

Within your application the trace command is not suitable for diagnostic. A
better source of information are the asynchronous notifications send by the camera. You can use the
``pcicclient`` module to receive those notifications and error messages. Please
take a look at the example code provided by
[ifm3d-examples](https://github.com/ifm/ifm3d-examples/tree/master/pcicclient_async_messages)
the anotated list of error codes is provided in the doc folder
[doc/pcic_async_errors.md](doc/pcic_async_errors.md).
