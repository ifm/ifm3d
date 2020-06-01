Installing ifm3d from source
----------------------------

### Build Dependencies

<table>
  <tr>
    <th>Dependency</th>
    <th>Dependent ifm3d module</th>
    <th>Notes</th>
  </tr>
  <tr>
    <td><a href="http://www.boost.org">Boost</a></td>
    <td>framegrabber, pcicclient, tools, pybind11</td>
    <td>We use Boost ASIO (header-only) to handle cross-platform network
     communication with the camera. While ASIO itself is header-only, it does
     require runtime linking to Boost System. We also use Boost Program Options
     to handle command line parsing in the tools module.</td>
  </tr>
  <tr>
    <td><a href="http://www.cmake.org">CMake</a></td>
    <td>camera, framegrabber, swupdater, image, opencv, pcicclient, tools,
    pybind11</td>
    <td>Meta-build framework</td>
  </tr>
  <tr>
    <td><a href="https://curl.haxx.se/libcurl">Curl</a></td>
    <td>tools, swupdater</td>
    <td>Used to help enable command-line based firmware flashing.</td>
  </tr>
  <tr>
    <td><a href="https://github.com/google/glog">Glog</a></td>
    <td>camera, framegrabber, swupdater, image, opencv, pcicclient, tools,
    pybind11</td>
    <td>Logging framework</td>
  </tr>
  <tr>
    <td><a href="https://github.com/google/googletest">Gtest</a></td>
    <td>camera, framegrabber, swupdater, image, opencv, pcicclient, tools</td>
    <td>Unit testing framework</td>
  </tr>
  <tr>
    <td><a href="http://xmlrpc-c.sourceforge.net/">libxmlrpc</a></td>
    <td>camera, pybind11</td>
    <td>XMLRPC client used call into the camera configuration interface</td>
  </tr>
  <tr>
    <td><a href="http://opencv.org">OpenCV</a></td>
    <td>image, opencv, pybind11</td>
    <td>N-dimensional array container for encoding 2d and 3d image data</td>
  </tr>
  <tr>
    <td><a href="http://pointclouds.org">PCL</a></td>
    <td>image</td>
    <td>A 3D point cloud encoding. NOTE: the PCL dependency in ifm3d is
    header-only (we need to construct a point cloud) however there is no
    runtime linking dependency.</td>
  </tr>
  <tr>
    <td><a href="https://github.com/pybind/pybind11">pybind11</a></td>
    <td>pybind11</td>
    <td>A header-only library that exposes C++ types in Python and vice versa,
    mainly to create Python bindings of existing C++ code.</td>
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
$ sudo apt-get update && sudo apt-get -y upgrade
$ sudo apt-get update && sudo apt-get install -y libboost-all-dev git jq libcurl4-openssl-dev \
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
$ sudo dpkg -i ifm3d_0.18.0_amd64-camera.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-swupdater.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-image.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-tools.deb
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
  associated [ROS wrapper](https://github.com/ifm/ifm3d-ros). If this is
  the case, you need to be sure that the version of OpenCV that you link to in
  both `ifm3d` and `ifm3d-ros` are consistent. To give you some control over
  that, the build process allows you to explicitly call out which version of
  OpenCV you wish to use. For example, if you are using OpenCV 2.4, your
  `cmake` line above should look something like:
  `$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DFORCE_OPENCV2=ON ..`. Similarly, if
  you are using OpenCV 3, your `cmake` line above should look something like:
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
$ sudo dpkg -i ifm3d_0.18.0_amd64-camera.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-swupdater.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-opencv.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-tools.deb
```

#### Building the Python Bindings

There are several options/configurations available for building and/or
installing the `pybind11` module. For this reason, building/installing the
`pybind11` module is not covered in this document. Please refer to the
[python](python.md) documentation for installation instructions.

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
$ sudo dpkg -i ifm3d_0.18.0_amd64-camera.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-swupdater.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-image.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-opencv.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-tools.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-pcicclient.deb
```
