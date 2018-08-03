ifm3d - Troubleshooting Guide
=============================

You can use this guide to help you identify and resolve basic problems you may be experiencing with ifm3d library or ifm 3d cameras.

# Select the troubleshooting options

- [Connecting the camera](#Connecting-the-camera)
- [ifm3d not found](#ifm3d-not-found)
 
## Connecting the camera 
While connecting to device through ifm3d e.g.
```
ifm3d ls
```
if you get following error message
``` 
ifm3d error: -100001
Lib: XMLRPC Timeout - can you `ping' the sensor? 
```
This shows that ifm3d is not able to connect to device on your network.

Following are some guidelines to troubleshoot this issue 
##### Provide the device IP address to the ifm3d
   
ifm3d by default connects to the default IP <"192.168.0.69">, one can also pass the 
IP through enviornment variable IFM3D_IP and for command line tools through --ip switch. 
To get the application list from camera one can use following command 
```
ifm3d --ip=192.168.0.69 ls 
```

##### Check if you can ping the camera

Power on the network and camera and check if you are able to ping the camera. If not successfull 
check for the physical connections.
 
##### If you are behind the proxy network 

Ping to device might not be successfull if you are under proxy network, to resolve this issue one needed to bypass the proxy 
for the device address. A quick fix is to set the environment variable **no_proxy** to the IP address of the device. 
if you are working with multiple device it is always good to bypass the required IP's in ~/.bashrc file as shown below
 
```
$ vi ~/.bashrc
no_proxy="127.0.0.1, localhost,  192.168.0.69, 192.168.0.70, 192.168.1.* " 
```

##### DHCP enabled device on DHCP network

On Windows, use IVA [**ifm Vision Assistant**](https://www.ifm.com/de/de/product/E3D300) software to search the device on the network and 
obtain the IP Address of device

On Linux,use following command 
```
 $ avahi-browse -arp 2>/dev/null | grep  ^=.*O3D.*Workstation 
```

## ifm3d not found 
This might occur if ifm3d is not installed properly.
##### Troubleshoot on Windows

Set the ifm3d executable path in the enviornment path of the system. This is explained very well in 
https://github.com/lovepark/ifm3d/blob/master/doc/windows.md#running-ifm3d-tool-on-windows 

##### Troubleshoot on Linux 

Use intruction at the https://github.com/lovepark/ifm3d#the-default-build to build and install the 
library on unix systems.