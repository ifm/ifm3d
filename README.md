
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
    <td>0.11.1</td>
    <td>1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522</td>
    <td>1.0.111, 1.0.122, 1.0.126</td>
    <td>16.04,18.04</td>
    <td>Bad pixels are now flagged by the driver, opencv image container module
    headers can be included in multiple translation units, and other minor bug
    fixes.</td>
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
</table>

As of version 0.9.0, we have removed the `viewer` sub-command from the `ifm3d`
command line tool (part of the `tools` module). The objective was to lessen the
dependencies for the core library. However, a *clone* of the pre-0.9.0
viewer is available in its own repository:
[ifm3d-pcl-viewer](https://github.com/lovepark/ifm3d-pcl-viewer).

Installing the Software
-----------------------

### Build Dependencies

<table>
  <tr>
    <th>Dependency</th>
    <th>Dependent ifm3d module</th>
    <th>Notes</th>
  </tr>
  <tr>
    <td><a href="http://www.boost.org">Boost</a></td>
    <td>framegrabber, pcicclient, tools</td>
    <td>We use Boost ASIO (header-only) to handle cross-platform network
     communication with the camera. While ASIO itself is header-only, it does
     require runtime linking to Boost System. We also use Boost Program Options
     to handle command line parsing in the tools module.</td>
  </tr>
  <tr>
    <td><a href="http://www.cmake.org">CMake</a></td>
    <td>camera, framegrabber, image, opencv, pcicclient, tools</td>
    <td>Meta-build framework</td>
  </tr>
  <tr>
    <td><a href="https://curl.haxx.se/libcurl">Curl</a></td>
    <td>tools</td>
    <td>Used to help enable command-line based firmware flashing.</td>
  </tr>
  <tr>
    <td><a href="https://github.com/google/glog">Glog</a></td>
    <td>camera, framegrabber, image, opencv, pcicclient, tools</td>
    <td>Logging framework</td>
  </tr>
  <tr>
    <td><a href="https://github.com/google/googletest">Gtest</a></td>
    <td>camera, framegrabber, image, opencv, pcicclient, tools</td>
    <td>Unit testing framework</td>
  </tr>
  <tr>
    <td><a href="http://xmlrpc-c.sourceforge.net/">libxmlrpc</a></td>
    <td>camera</td>
    <td>XMLRPC client used call into the camera configuration interface</td>
  </tr>
  <tr>
    <td><a href="http://opencv.org">OpenCV</a></td>
    <td>image, opencv</td>
    <td>N-dimensional array container for encoding 2d and 3d image data</td>
  </tr>
  <tr>
    <td><a href="http://pointclouds.org">PCL</a></td>
    <td>image</td>
    <td>A 3D point cloud encoding. NOTE: the PCL dependency in ifm3d is
    header-only (we need to construct a point cloud) however there is no runtime
    linking dependency.</td>
  </tr>
</table>

Additionally, if you plan to build the debian packages and have the
dependencies computed for you dynamically (see the note below on the
`repackage` target), you will also need:

* [Python 2.7](https://www.python.org/)
* [readelf](https://www.gnu.org/software/binutils/) (Part of the `binutils` package)
* [ldd](http://man7.org/linux/man-pages/man1/ldd.1.html) (Part of the `libc-bin` package)
* [dpkg](https://help.ubuntu.com/lts/serverguide/dpkg.html)

We note that, if you are running on a supported Linux, all of these packages
are available through the offical debian repositories and should be a simple
`apt-get` away from being installed on your machine.

Use the following steps to install all the library dependencies on Debian based
systems

```
$ sudo apt-get update && apt-get -y upgrade
$ sudo apt-get update && apt-get install -y libboost-all-dev git jq libcurl4-openssl-dev \
                                            libgtest-dev libgoogle-glog-dev  \
                                            libxmlrpc-c++8-dev libopencv-dev \
                                            libpcl-dev libproj-dev \
                                            build-essential coreutils cmake
```
Note: The package name may differ in different flavours of Linux. Above apt-get commands
are specific to Debian based systems

### Building From Source

#### The default build

By default, the `ifm3d` build enables the `camera`, `framegrabber`, `image`,
and `tools` modules. Building the software follows the usual cmake idiom of:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr ..
$ make
$ make check
$ sudo make install
```

Alternatively, if you are on a supported Linux platform (see above), the
preferred method of building and installing the software is:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr ..
$ make
$ make check
$ make package
$ make repackage
$ sudo dpkg -i ifm3d_0.9.0_amd64-camera.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-image.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-tools.deb
```

(The version number embedded in the deb file will be dependent upon which
version of the `ifm3d` software you are building)

A few important notes when building from source:

* For the `make check` step, you will need to have your camera plugged in. The
  camera settings will get mutated by this process, so, you are encouraged to
  back up your configuration if you'd like to later restore your camera to its
  pre-testing state. You are also encouraged to test against the camera (or
  cameras) you plan to use. I.e., O3D, O3X, etc.

* Many `ifm3d` users ultimately plan to use this library along with its
  associated [ROS wrapper](https://github.com/lovepark/ifm3d-ros). If this is
  the case, you need to be sure that the version of OpenCV that you link to in
  both `ifm3d` and `ifm3d-ros` are consistent. To give you some control over
  that, the build process allows you to explicitly call out which version of
  OpenCV you wish to use. For example, if you are using OpenCV 2.4, your
  `cmake` line above should look something like:
  `$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DFORCE_OPENCV2=ON ..`. Similarly, if you using
   OpenCV 3, your `cmake` line above should look something like:
   `$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DFORCE_OPENCV3=ON ..`

* Experienced users may be puzzled by the `repackage` step. If you are simply
  building for your local machine, you can skip it (albeit, with minimal
  risk). This step is used to dynamically compute the debian dependencies for
  the particular module. Due to how we are partitioning out the software, this
  approach is necessary vs. the more traditional
  `CPACK_DEBIAN_PACKAGE_SHLIBDEPS` wrapper around `dpkg-shlibdeps`. We
  basically created [a version of that tool](cmake/utils/ifm3d-dpkg-deps.py.in)
  that exploits *a-priori* information about the `ifm3d` environment to
  properly compute the debian dependencies. If you are building debs on a build
  machine to be distributed out to various runtime computers, you will
  certainly want to execute the `repackage` target so that you are ensured the
  runtime machines have the proper dependency chain in place.

#### A build that does not require PCL

Many `ifm3d` users seem to be moving away from PCL. To that end, it is possible
to build `ifm3d` without any reliance on PCL yet still maintain a bridge to
OpenCV image encodings. (NOTE: As of 0.9.0, it is also quite easy to build your
own image encoding to interoperate within the `ifm3d` ecosystem.)

Building `ifm3d` without a PCL dependency looks like the following:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_MODULE_OPENCV=ON -DBUILD_MODULE_IMAGE=OFF ..
$ make
$ make check
$ make package
$ make repackage
$ sudo dpkg -i ifm3d_0.9.0_amd64-camera.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-opencv.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-tools.deb
```

#### A sumo-build

If you want to build everything:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_MODULE_OPENCV=ON -DBUILD_MODULE_PCICCLIENT=ON ..
$ make -j 8
$ make check
$ make package
$ make repackage
$ sudo dpkg -i ifm3d_0.9.0_amd64-camera.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-image.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-opencv.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-tools.deb
$ sudo dpkg -i ifm3d_0.9.0_amd64-pcicclient.deb
```

Additional Resources
--------------------
* [Building on Windows](doc/windows.md)
* [Basic library usage](doc/basic_usage.md)
* [ifm3d command line tool](doc/cmdline.md)
* [Configuring Your Camera](doc/configuring.md)
* [Viewing the Point Cloud](https://github.com/lovepark/ifm3d-pcl-viewer)
* [Implementing your own image container](doc/img_container.md)
* [ROS](https://github.com/lovepark/ifm3d-ros)
* [Troubleshoot](doc/troubleshoot.md)


Known Issues, Bugs, and our TODO list
-------------------------------------
Please see the [Github Issue Tracker](https://github.com/lovepark/ifm3d/issues).


LICENSE
-------
Please see the file called [LICENSE](LICENSE).
