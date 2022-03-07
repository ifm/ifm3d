## Installing ifm3d from source
### Overview of the available build flags
| Flag name | Description | Default value |
| --------- | ----------- | ------------- |
| BUILD_MODULE_FRAMEGRABBER | Build the framegrabber module | ON | 
| BUILD_MODULE_STLIMAGE | Build the stl image module (Only relies on standard c++ libraries) | ON | 
| BUILD_MODULE_IMAGE **DEPRECATED**| Build the image module (Depends on OpenCV and PCL) | OFF | 
| BUILD_MODULE_OPENCV **DEPRECATED**| Build the OpenCV-only image container | OFF |
| BUILD_MODULE_TOOLS | Build the command-line utility | ON |  
| BUILD_IN_DEPS | Download and build dependencies | ON | 
| BUILD_MODULE_PYBIND11 | Build the ifm3dpy python package (it can also be installed directly through `pip`) | OFF | 
| USE_LEGACY_COORDINATES | Use the legacy coordinates (ifm3d <= 0.92.x) with swapped axis | OFF | 
| BUILD_MODULE_SWUPDATER | Build the swupdater module | ON | 
| BUILD_SDK_PKG | Build install packages for development purposes | ON | 
| FORCE_OPENCV3 | Force the build to require OpenCV 3 | OFF | 
| FORCE_OPENCV2 | Force the build to require OpenCV 2.4 | OFF | 
| BUILD_SHARED_LIBS | Build modules as shared libraries | ON | 
| BUILD_EXAMPLES | Build the examples | OFF | 
| BUILD_DOC | Build documentation | OFF | 
| BUILD_TESTS | Build unit tests | ON
| BUILD_MODULE_PCICCLIENT | Build the pcicclient module | OFF | 
### Build Dependencies

<table>
  <tr>
    <th>Dependency</th>
    <th>Dependent ifm3d module</th>
    <th>Notes</th>
  </tr>
  <tr>
    <td><a href="http://www.cmake.org">CMake</a></td>
    <td>camera, framegrabber, swupdater, image, opencv, stlimage, pcicclient, tools,
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
    <td>camera, framegrabber, swupdater, image, opencv, stlimage, pcicclient, tools,
    pybind11</td>
    <td>Logging framework</td>
  </tr>
  <tr>
    <td><a href="https://github.com/google/googletest">Gtest</a></td>
    <td>camera, framegrabber, swupdater, image, opencv, stlimage, pcicclient, tools</td>
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

Use the following steps to install all the library dependencies on Debian based systems

```
$ sudo apt-get update && sudo apt-get -y upgrade
$ sudo apt-get update && sudo apt-get install -y 
      git \ 
      jq \ 
      libcurl4-openssl-dev \
      libgtest-dev libgoogle-glog-dev  \
      libxmlrpc-c++8-dev \ 
      libproj-dev \
      build-essential \
      coreutils \
      cmake
# Only if you wish to build the image and/or opencv modules
$ sudo apt-get update && sudo apt-get install -y libopencv-dev libpcl-dev                 
# Only if you wish to build the python bindings
$ sudo apt-get update && sudo apt-get install pybind11-dev                          
```
Note: The package name may differ in different flavours of Linux. 
Above apt-get commands are specific to Debian based systems

### Building From Source
Start with cloning the code from the ifm3d github repository {{ '[here]({})'.format(ifm3d_gh_url) }}.  

⚠ The code on the branch {{ ifm3d_main_branch }} is updated nightly and contains the latest changes to the library. It is typically a work in progress.  
⚠ We recommend using tagged versions for your builds, to ensure consistency between builds. The latest tagged version can be found {{ '[here]({})'.format(ifm3d_latest_tag_url) }}.

#### The default build

By default, the `ifm3d` build enables the `camera`, `framegrabber`, `stlimage`,
and `tools` modules. Building the software follows the usual cmake idiom of:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr ..
$ cmake --build .
$ sudo cmake --build . --target install
```

#### Building with PCL and/or OPENCV

`ifm3d` provides multiple image buffers. The default one, `stlimage` only relies on standard c++ libraries. The `image` module relies on opencv and pcl. The `opencv` modules relies only on openCV. To build either of these use the following:
```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_MODULE_OPENCV=ON  -DBUILD_IN_DEPS=OFF ..
# OR
$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_MODULE_IMAGE=ON -DBUILD_IN_DEPS=OFF ..
$ cmake --build .
$ sudo cmake --build . --target install
```

> Note: Many `ifm3d` users ultimately plan to use this library along with its associated [ROS wrapper](ROS/index:ROS%20wrappers%20for%20ifm3d).
> If this is the case, you need to be sure that the version of OpenCV that you link to in both `ifm3d` and `ifm3d-ros` are consistent. 
> To give you some control over that, the build process allows you to explicitly call out which version of OpenCV you wish to use. 
> For example, if you are using OpenCV 2.4, your `cmake` line above should look something like: `$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DFORCE_OPENCV2=ON ..`. 
> Similarly, if you are using OpenCV 3, your `cmake` line above should look something like: `$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DFORCE_OPENCV3=ON ..`

#### Building the Python Bindings

There are several options available for building and/or installing the `ifm3dpy` module. The most simple one is to install with pip:
```
pip install ifm3dpy
```
For more options, please refer to the [python](python.md) documentation.

#### A sumo-build

If you want to build everything:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_MODULE_OPENCV=ON -DBUILD_MODULE_PCICCLIENT=ON -DBUILD_MODULE_IMAGE=ON -DBUILS_IN_DEPS=OFF ..
$ cmake --build .
$ sudo cmake --build . --target install
```
#### Building the examples

The examples can be built along with the rest of the library by switching the proper flag on. Assuming you are in the /build folder:

```
$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_O3R_EXAMPLES=ON ..
$ make
```

#### Building debian packages

Alternatively, to build *debs* to be distributed to multiple runtime machines, you can use the following:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr ..
$ cmake --build .
$ cmake --build . package
$ cmake --build . repackage
$ sudo dpkg -i ifm3d_0.18.0_amd64-camera.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-swupdater.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-stlimage.deb
$ sudo dpkg -i ifm3d_0.18.0_amd64-tools.deb
```

(The version number embedded in the deb file will be dependent upon which
version of the `ifm3d` software you are building)

> Note: Experienced users may be puzzled by the `repackage` step. 
> This step is used to dynamically compute the debian dependencies for the particular module. 
> Due to how we are partitioning out the software, this approach is necessary vs. the more traditional `CPACK_DEBIAN_PACKAGE_SHLIBDEPS` wrapper around `dpkg-shlibdeps`. 
> We basically created [a version of that tool](cmake/utils/ifm3d-dpkg-deps.py.in) that exploits *a-priori* information about the `ifm3d` environment to properly compute the debian dependencies. 
> If you are building debs on a build machine to be distributed out to various runtime computers, you will certainly want to execute the `repackage` target so that you are ensured the runtime machines have the proper dependency chain in place.
