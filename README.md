
ifm3d
=====
Library and utilities for working with ifm pmd-based 3D ToF Cameras.


Software Compatibility Matrix
-----------------------------
<table>
  <tr>
    <th>ifm3d version</th>
    <th>O3D Firmware Version</th>
    <th>O3X Firmware Version</th>
    <th>Notes</th>
  </tr>
  <tr>
    <td>0.1.0</td>
    <td>1.6.2114</td>
    <td>0.1.4</td>
    <td>Initial (beta) release</td>
  </tr>
</table>

Disclaimer
----------
This software is under active development and not yet ready for production
systems. If you are looking for a C++ sensor interface to the ifm O3D303,
please consider using [libo3d3xx](https://github.com/lovepark/libo3d3xx).

If you are an early-adopter user of the new O3X, please keep reading.


Organization of the Software
----------------------------
The ifm3d software is organized into modules, they are:

<table>
  <tr>
    <th>Module Name</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>Camera</td>
    <td>Provides an implementation of the XMLRPC protocol for configuring the
  camera and PMD imager settings</td>
  </tr>
  <tr>
    <td>Framegrabber</td>
    <td>Provides an implementation of the PCIC protocol for streaming pixel
  data and triggered image acquisition.</td>
  </tr>
  <tr>
    <td>Image</td>
    <td>Provides a bridge from raw camera bytes to OpenCV and PCL image
  encodings.</td>
  </tr>
  <tr>
    <td>Tools</td>
    <td>Provides the ifm3d command line tool for manipulating and introspecting
  the hardware interactively. It is also suitable for usage within shell scripts.</td>
  </tr>
</table>


Installing the Software
-----------------------

### Building from source

### Installing binaries

If you are running on Ubuntu (16.04 LTS) and running ROS (Kinetic), binaries
have been supplied with the initial 0.1.0 release. They can be downloaded from
[here](https://github.com/lovepark/ifm3d/releases/tag/v0.1.0)

Before installing the supplied debs, you should first update your packages on
Ubuntu 16.04 LTS:

    $ sudo apt-get update
    $ sudo apt-get -u upgrade

You should then
[install ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu). We recommend
the `ros-kinetic-desktop-full` installation.

**NOTE:** ROS is not strictly necessary for using ifm3d. We recommend
  installing ROS when using our supplied binaries because (currently) our
  dependency on OpenCV in the Debian control file is expressed as the version
  of OpenCV 3 supplied with ROS Kinetic, namely
  `ros-kinectic-opencv3`. Experienced users can work around this restriction.

Once you have downloaded the supplied binaries, they can be installed as
follows (please note the order of installation):

    $ sudo dpkg -i ifm3d_0.1.0_amd64-camera.deb
    $ sudo dpkg -i ifm3d_0.1.0_amd64-framegrabber.deb
    $ sudo dpkg -i ifm3d_0.1.0_amd64-image.deb
    $ sudo dpkg -i ifm3d_0.1.0_amd64-tools.deb

If you run into a dependency issues, e.g., on a fresh Ubuntu install you may
see something like:

```
$ sudo dpkg -i ifm3d_0.1.0_amd64-camera.deb
Selecting previously unselected package ifm3d-camera.
(Reading database ... 249133 files and directories currently installed.)
Preparing to unpack ifm3d_0.1.0_amd64-camera.deb ...
Unpacking ifm3d-camera (0.1.0) ...
dpkg: dependency problems prevent configuration of ifm3d-camera:
 ifm3d-camera depends on libgoogle-glog0v5; however:
  Package libgoogle-glog0v5 is not installed.
 ifm3d-camera depends on libxmlrpc-c++8v5; however:
  Package libxmlrpc-c++8v5 is not installed.
```

You can do the following to resolve this:

    $ sudo apt-get -f install

At this point, if your camera is plugged in and powered on, you should be able
to list its applications. For example with an O3X plugged in:

```
$ ifm3d ls
[
  {
    "Active": true,
    "Description": "",
    "Id": 1299148885,
    "Index": 1,
    "Name": ""
  }
]
```

For viewing the point cloud data, currently, our only supported visualization
tool is [rviz](http://wiki.ros.org/rviz). To utilize it, you will need to
install our [ifm3d ROS bindings](https://github.com/lovepark/ifm3d-ros).


Known Issues, Bugs, and our TODO list
-------------------------------------

We greatly appreciate your choice to use our sensor interface software. As of
this writing, this software is very new. However, it is informed by many years
of developing and supporting
[libo3d3xx](https://github.com/lovepark/libo3d3xx), the de-facto C++ sensor
interface to the ifm O3D. We are committed to evolving this software package
into the production quality code that has made libo3d3xx the basis of many
commercial products with the additional benefit of providing a hardware
abstraction layer over top of *all* ifm pmd-based 3D ToF cameras.

We appreciate your feedback as you use the code. We will be tracking things
(including our TODO list) on the
[Github Issue Tracker](https://github.com/lovepark/ifm3d/issues).

LICENSE
-------

Please see the file called [LICENSE](LICENSE).

AUTHORS
-------

Tom Panzarella <tom@loveparkrobotics.com>
