
# ifm3d Overview

Library and utilities for working with ifm pmd-based 3D ToF Perception devices. Compatible with the O3R, O3D and O3X platforms. 
This library is available in python, c++, and has wrappers for ROS and ROS2.
Comprehensive documentation is available on [ifm3d.com](https://ifm3d.com/).

![Build (Ubuntu)](https://github.com/ifm/ifm3d/workflows/Build%20(Ubuntu)/badge.svg?branch=master)
![Build (Windows)](https://github.com/ifm/ifm3d/workflows/Build%20(Windows)/badge.svg?branch=master)

| 3D cloud | Distance | RGB |
| -- | -- | -- |
| ![3D cloud of a stack of boxes](xyz.png) | ![Distance image of a stack of boxes](distance.png) | ![RGB image of a stack of boxes](jpeg.png) |
## Released Versions

⚠️ Note that the `master` branch is generally in a work in progress state and you probably want to use a
tagged [release version](https://github.com/ifm/ifm3d/releases) for production.

⚠️ Branch `o3r/main-next` is an early adopters version which may/will contain changes to the interface in the future.

### Current Revision

<table>
  <tr>
    <th>ifm3d version</th>
    <th>Supported O3D Firmware Version</th>
    <th>Supported O3X Firmware Version</th>
    <th>Supported O3R Firmware Version</th>
    <th>Supported Ubuntu Linux Version</th>
    <th>Notes</th>
  </tr>
  <tr>
    <td>0.92.0, 0.93.0 </td>
    <td>1.6.2114, 1.23.1522, 1.23.1522, 1.23.2848, 1.30.4123, 1.30.5309</td>
    <td>1.0.122, 1.0.126, 1.0.156, 1.1.190</td>
    <td>0.13.11</td>
    <td>16.04,18.04,20.04</td>
    <td>Removed Boost dependency, Added clang-format, SPDX license Headers</td>
  </tr>
</table>

## Organization of the Software

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
    <td>image - DEPRECATED</td>
    <td>Provides a bridge from raw camera bytes to OpenCV and PCL image encodings.</td>
  </tr>
  <tr>
    <td>opencv - DEPRECATED</td>
    <td>This is an officially supported and alternate data container to the
    STLImage module. This module provides a bridge from raw camera bytes
    to OpenCV image encodings without any dependence upon PCL.</td>
  </tr>
  <tr>
    <td>stlimage</td>
    <td>This is the default data container, replacing the
    deprecated Image module. This module provides a STL Image container without any
    third-party dependencies</td>
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

## Installation instructions
Please refer to the corresponding section on [ifm3d.com](https://ifm3d.com/).

## Supported docker containers
Docker containers are available for the ifm3d library, both on [ghcr](https://github.com/orgs/ifm/packages?repo_name=ifm3d) and on the [dockerhub](https://hub.docker.com/r/ifmrobotics/ifm3d). 
You can pull them with:
```bash
docker pull ghcr.io/ifm/ifm3d:stable
```
OR
```bash
docker pull ifmrobotics/ifm3d:stable
```
Note that we provide 2 tags, *stable* always points to the latest tagged version, and *latest* is built nightly with the latest changes on the o3r/main-next branch. The *latest* tag is typically a work in progress.

## Report a bug and check the known issues

Please see the [Github Issue Tracker](https://github.com/ifm/ifm3d/issues).

## LICENSE

Please see the [LICENSE](LICENSE) file.
