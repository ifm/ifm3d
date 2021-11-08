
# ifm3d Overview

Library and utilities for working with ifm pmd-based 3D ToF Cameras.

![Build (Ubuntu)](https://github.com/ifm/ifm3d/workflows/Build%20(Ubuntu)/badge.svg?branch=master)
![Build (Windows)](https://github.com/ifm/ifm3d/workflows/Build%20(Windows)/badge.svg?branch=master)

## Release versions

⚠️ Note that the `master` branch is generally in a work in progress state and you probably want to use a
tagged [release version](https://github.com/ifm/ifm3d/releases) for production.

⚠️ branch `o3r/main` is an early adopters version which may/will contain changes to the interface in the future.

## Current Revision

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
    <td>0.92.0 </td>
    <td>1.6.2114, 1.23.1522, 1.23.1522, 1.23.2848, 1.30.4123, 1.30.5309</td>
    <td>1.0.122, 1.0.126, 1.0.156</td>
    <td>0.13.11</td>
    <td>16.04,18.04,20.04</td>
    <td>Removed Boost dependence,Added clang-format,SPDX license Headers</td>
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
    <td>opencv</td>
    <td>This is an officially supported and alternate data container to the
    default Image module. This module provides a bridge from raw camera bytes
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

As of version 0.9.0, we have removed the `viewer` sub-command from the `ifm3d`
command line tool (part of the `tools` module). The objective was to lessen the
dependencies for the core library. However, a *clone* of the pre-0.9.0
viewer is available in its own repository:
[ifm3d-pcl-viewer](https://github.com/ifm/ifm3d-pcl-viewer).

## Additional Resources 
* [Viewing the Point Cloud](https://github.com/ifm/ifm3d-pcl-viewer)
* [ROS](https://github.com/ifm/ifm3d-ros)
* [ROS 2](https://github.com/ifm/ifm3d-ros2)

## Known Issues, Bugs, and our TODO list

Please see the [Github Issue Tracker](https://github.com/ifm/ifm3d/issues).


## LICENSE

Please see the file called [LICENSE](LICENSE).
