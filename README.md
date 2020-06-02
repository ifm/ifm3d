
ifm3d
=====
Library and utilities for working with ifm pmd-based 3D ToF Cameras.

![Build (Ubuntu)](https://github.com/ifm/ifm3d/workflows/Build%20(Ubuntu)/badge.svg?branch=master)
![Build (Windows)](https://github.com/ifm/ifm3d/workflows/Build%20(Windows)/badge.svg?branch=master)

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
    <td>0.18.0 </td>
    <td>1.6.2114, 1.23.1522, 1.23.1522, 1.23.2848, 1.30.4123, 1.30.5309</td>
    <td>1.0.122, 1.0.126, 1.0.156</td>
    <td>16.04,18.04,20.04</td>
    <td>Expanded support matrix for platforms/archs/firmwares</td>
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
[ifm3d-pcl-viewer](https://github.com/ifm/ifm3d-pcl-viewer).

Installing the Software
-----------------------
Binaries for ifm3d are available on a few supported platforms. Instructions for
each now follow.

## Linux

#### Snap Application

[![ifm3d](https://snapcraft.io//ifm3d/badge.svg)](https://snapcraft.io/ifm3d)

The ifm3d [command line utility](doc/cmdline.md) and
[viewer tool](https://github.com/ifm/ifm3d-pcl-viewer) are available as a snap
in the Snapcraft application store. Snapcraft supports [most major Linux
distributions](https://snapcraft.io/docs/installing-snapd). The storefront
page for `ifm3d` is: https://snapcraft.io/ifm3d

To install the package on a snapd enabled system:
```
$ sudo snap install ifm3d
```
After installation, the commands `ifm3d` and `ifm3d.viewer` should be
available.

**NOTE**: This option is a convenient way to install and update tools and
utilities for working with ifm Cameras. This option is not suitable for
developers hoping to target the `ifm3d` API and shared libraries. Developers
should opt for standard debian packages, or building from source.

#### Ubuntu Linux via Apt (amd64/arm64)

We provide apt repositories for the following Ubuntu Linux distributions and
architectures:

<table>
  <tr>
    <th/>
    <th>Ubuntu 16.04 Xenial</th>
    <th>Ubuntu 18.04 Melodic</th>
    <th>Ubuntu 20.04 Focal</th>
  </tr>
  <tr>
    <th>amd64</th>
    <td>X</td>
    <td>X</td>
    <td>X</td>
  </tr>
  <tr>
    <th>arm64</th>
    <td>X</td>
    <td>X</td>
    <td>X</td>
  </tr>
</table>

Add the repository to your sources.list:
```
$ sudo sh -c 'echo "deb https://nexus.ifm.com/repository/ifm-robotics_ubuntu_$(lsb_release -sc)_$(dpkg --print-architecture) $(lsb_release -sc) main" > /etc/apt/sources.list.d/ifm-robotics.list'
```

Add the public key for the repository:
```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 8AB59D3A2BD7B692
```

Install the software:

```
$ sudo apt-get update
$ sudo apt-get install ifm3d-camera \
                       ifm3d-framegrabber \
                       ifm3d-swupdater \
                       ifm3d-image \
                       ifm3d-opencv \
                       ifm3d-pcicclient \
                       ifm3d-tools \
                       ifm3d-python3 \
                       ifm3d-pcl-viewer \
```

#### Linux for Tegra

Linux for Tegra is an NVIDIA Linux distribution for the Jetson family of GPU
SoC systems. NVIDIA distributes a software package called JetPack with various
utilities and libraries optimized for the target hardware. There are a few
pakages which override the core Ubuntu packages (`OpenCV` as the primary
example). We provide alternate apt repositories for `ifm3d` built on top of
the JetPack libraries rather than the Ubuntu libraries.

Add *one* of the following repositories based on your desired JetPack/L4T
release:

Jetpack 4.4:
```
$ sudo sh -c 'echo "deb https://nexus.ifm.com/repository/ifm-robotics_l4t_jetpack_4_4_arm64 melodic main" > /etc/apt/sources.list.d/ifm-robotics.list'
```
Jetpack 4.3:
```
$ sudo sh -c 'echo "deb https://nexus.ifm.com/repository/ifm-robotics_l4t_jetpack_4_3_arm64 melodic main" > /etc/apt/sources.list.d/ifm-robotics.list'
```

Add the public key for the repository:
```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 8AB59D3A2BD7B692
```

Install the software:

```
$ sudo apt-get update
$ sudo apt-get install ifm3d-camera \
                       ifm3d-framegrabber \
                       ifm3d-swupdater \
                       ifm3d-image \
                       ifm3d-opencv \
                       ifm3d-pcicclient \
                       ifm3d-tools \
                       ifm3d-python3 \
                       ifm3d-pcl-viewer \
```

#### ROS/ROS2

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

### Windows

Pre-built binaries are not yet available. For now we recommend manually
compiling for the target MSVC/SDK versions according to the
[building on windows](doc/windows.md) instructions.

### Other platforms

If you are running on a platform we did not mention above, the links below will
assist you in building from source code. Alternatively, if you are simply
looking to do a quick evaluation of an `ifm3d` supported sensor we recommend
either the Snapcraft option discussed above, or [Docker containers](docker/).


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
