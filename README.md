
ifm3d
=====
Library and utilities for working with ifm pmd-based 3D ToF Cameras.

Current Revision
----------------
<table>
  <tr>
    <th>ifm3d version</th>
    <th>Supported O3D Firmware Version</th>
    <th>Supported O3X Firmware Version</th>
    <th>Supported Ubuntu Linux Version</th>
    <th>Notes</th>
  </tr>
  <tr>
    <td>0.17.0 </td>
    <td>1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522, 1.23.2848,
    1.25.4073, 1.30.4123</td>
    <td>1.0.111, 1.0.122, 1.0.126</td>
    <td>16.04,18.04</td>
    <td>Bugfixes and removed FrameGrabberUdp module</td>
  </tr>
</table>

A full software compatibility matrix, including older releases, is available [here](doc/swcompat.md).

Organization of the Software
----------------------------
The ifm3d software is organized into modules, they are:

<table>
  <tr>
    <th>Module Name</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>camera</td>
    <td>Provides an implementation of the XMLRPC protocol for configuring the
    camera and pmd imager settings.</td>
  </tr>
  <tr>
    <td>framegrabber</td>
    <td>Provides an implementation of the PCIC protocol for streaming pixel
    data and triggered image acquisition.</td>
  </tr>
  <tr>
    <td>swupdater</td>
    <td>Provides utilities for managing the SWUpdate subsystem of the
    camera.</td>
  </tr>
  <tr>
    <td>image</td>
    <td>Provides a bridge from raw camera bytes to OpenCV and PCL image encodings.</td>
  </tr>
  <tr>
    <td>opencv</td>
    <td>This is an officially supported and alternate data container to the
    default Image module. This module provides a bridge from raw camera bytes
    to OpenCV image encodings without any dependence upon PCL.</td>
  </tr>
  <tr>
    <td>pcicclient</td>
    <td>Direct access to PCIC to, for example, actuate digital IO.</td>
  </tr>
  <tr>
    <td>tools</td>
    <td>Provides the ifm3d command line tool for manipulating and introspecting
    the hardware interactively. It is also suitable for usage within shell
    scripts to, for example, manage fleets of cameras.</td>
  </tr>
  <tr>
    <td>pybind11</td>
    <td>Provides python bindings through
    <a href="https://github.com/pybind/pybind11">pybind11</a> to the native C++ API.
    Supports all general camera functionality as well as a zero-copy interface
    to image data, exposed as NumPy arrays.</td>
  </tr>
</table>

As of version 0.9.0, we have removed the `viewer` sub-command from the `ifm3d`
command line tool (part of the `tools` module). The objective was to lessen the
dependencies for the core library. However, a *clone* of the pre-0.9.0
viewer is available in its own repository:
[ifm3d-pcl-viewer](https://github.com/lovepark/ifm3d-pcl-viewer).

Installing the Software
-----------------------
Binaries for ifm3d are available on a few supported platforms. Instructions for
each now follow.

### Linux

#### Configure the ifm apt server

We are currently supporting binaries for the two most recent LTS releases of
Ubuntu Linux. The first step in installation is to set up your computer to
accept software from ifm's apt server. The apt repository you want to point to
will depend on your version of Ubuntu and whether or not you plan to utilize
the [ifm3d ROS bindings](https://github.com/ifm/ifm3d-ros). Please follow the
instructions appropriate for your machine below:

##### Ubuntu 16.04, no ROS

```
$ sudo sh -c 'echo "deb [arch=amd64] https://nexus.ifm.com/repository/ifm-robotics_ubuntu_xenial_amd64 xenial main" > /etc/apt/sources.list.d/ifm-robotics.list'
```

##### Ubuntu 18.04, no ROS

```
$ sudo sh -c 'echo "deb [arch=amd64] https://nexus.ifm.com/repository/ifm-robotics_ubuntu_bionic_amd64 bionic main" > /etc/apt/sources.list.d/ifm-robotics.list'
```

##### Ubuntu 16.04, ROS Kinetic

```
$ sudo sh -c 'echo "deb [arch=amd64] https://nexus.ifm.com/repository/ifm-robotics_ubuntu_xenial_amd64_ros xenial main" > /etc/apt/sources.list.d/ifm-robotics.list'
```

##### Ubuntu 18.04, ROS Melodic

```
$ sudo sh -c 'echo "deb [arch=amd64] https://nexus.ifm.com/repository/ifm-robotics_ubuntu_bionic_amd64_ros bionic main" > /etc/apt/sources.list.d/ifm-robotics.list'
```

#### Set up your keys

```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 8AB59D3A2BD7B692
```

#### Install

Update your package index:

```
$ sudo apt-get update
```

Install the software:

```
$ sudo apt-get install ifm3d-camera \
                       ifm3d-framegrabber \
                       ifm3d-swupdater \
                       ifm3d-image \
                       ifm3d-opencv \
                       ifm3d-pcicclient \
                       ifm3d-tools \
                       ifm3d-python \
                       ifm3d-python3
```

If you are on a non-ROS platform, you can also install the default non-ROS
visualizer. If you are running ROS, we assume you will use rviz.

```
$ sudo apt-get install ifm3d-pcl-viewer
```

For ROS users, at this time, you still need to build the ROS node from
source. Those instructions are located at the
[ifm3d-ros](https://github.com/ifm/ifm3d-ros) project page. The instructions
above will simply install the core `ifm3d` sensor interface and tools, linked
properly against libraries in your ROS environment.

### Windows

Coming soon...

### Other platforms

If you are running on a platform we did not mention above, the links below will
assist you in building from source code. Alternatively, if you are simply
looking to do a quick evaluation of an `ifm3d` supported sensor, you can try
one of our experimental (as of this writing) Docker containers. We have options
for both ROS and non-ROS users. More information is located [here](docker/).


Additional Resources
--------------------
* [Building from source on Linux](doc/source_build.md)
* [Building on Windows](doc/windows.md)
* [Building and using Python bindings](doc/python.md)
* [Basic library usage](doc/basic_usage.md)
* [ifm3d command line tool](doc/cmdline.md)
* [Configuring Your Camera](doc/configuring.md)
* [Viewing the Point Cloud](https://github.com/ifm/ifm3d-pcl-viewer)
* [Implementing your own image container](doc/img_container.md)
* [ROS](https://github.com/ifm/ifm3d-ros)
* [ROS 2](https://github.com/ifm/ifm3d-ros2)
* [Troubleshoot](doc/troubleshoot.md)
* [Computing Cartesian data off-board](doc/compute_cartesian/O3D_Cartesian_Computation.ipynb) using the
  `ifm3dpy` Python bindings.

Known Issues, Bugs, and our TODO list
-------------------------------------
Please see the [Github Issue Tracker](https://github.com/ifm/ifm3d/issues).


LICENSE
-------
Please see the file called [LICENSE](LICENSE).
