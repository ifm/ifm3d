# Building ifm3d from source on Windows

This tutorial details how to compile the ifm3d library and its dependencies on
a Windows platform using Visual Studio.

# Dependencies

## Build tools

* [CMake v3.5.0](http://www.cmake.org) or newer (also available through the
Visual Studio installer)
* [Git for Windows](https://gitforwindows.org) (also available through the
Visual Studio installer)
* [Microsoft Visual Studio](https://www.visualstudio.com)
version 2015, 2017 or 2019. The free 'Community' edition is sufficient. Be
sure to select the 'Desktop development with C++' workflow.

## Binary Dependencies

### [PCL](http://www.pointclouds.org)

PCL is available in binary form for Windows platforms via the project's GitHub
releases page. `ifm3d` is tested against version v1.8.1 which can be
downloaded and installed via the following links.

Choose ONE of the following based on your target version of Visual Studio.
 * Visual Studio 2015: [PCL-1.8.1-AllInOne-msvc2015-win64.exe](https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2015-win64.exe)
 * Visual Studio 2017/2019: [PCL-1.8.1-AllInOne-msvc2017-win64.exe](https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2017-win64.exe)

**NOTE**: Opt-in to installing the 3rd party dependencies of `PCL`. `ifm3d`
also takes a dependency on the `boost` library. For simplicity/compatibility,
this tutorial builds against the version of `boost` provided along with
`PCL`.

**NOTE**: It is possible to build an ifm3d variant which does not depend on the
`PCL` library. See [Appendix A](#appendex-a) below for instructions.

### [OpenCV 3.4](https://opencv.org/)

OpenCV 3.4 is available in binary form for Windows platforms from the [OpenCV
Releases](https://opencv.org/releases/) page. `ifm3d` is tested against
v3.4.9, available from the following link:

* [OpenCV 3.4.9](https://sourceforge.net/projects/opencvlibrary/files/3.4.9/opencv-3.4.9-vc14_vc15.exe/download)

Download it and extract to a known location (this tutorial assumes a path of
`C:\opencv`.

## Source Dependencies

`ifm3d` depends on several additional libraries (curl, xmlrpc-c, glog, and
gtest) which are not available as binary packages on Windows. The following
instructions detail how to compile them from source for your target.

### Environment Configuration

The following environment variables are used by this tutorial to make
customization simpler. Modify them as needed for your environment. You can
obtain a list of valid cmake generator strings via `cmake -h`. Again, `ifm3d`
supports version 2015 and newer.

```
set IFM3D_BOOST_ROOT="C:\Program Files\PCL 1.8.1\3rdParty\Boost"
set IFM3D_OPENCV_PATH=C:\opencv\build
set IFM3D_CMAKE_GENERATOR="Visual Studio 15 2017 Win64"
set IFM3D_BUILD_DIR=C:\ifm3d
set CONFIG=Release
```

Finally, create the working directory in which `ifm3d` and its dependencies will
be built:
```
mkdir %IFM3D_BUILD_DIR%
```

### [curl](https://curl.haxx.se/)
```
cd %IFM3D_BUILD_DIR%
git clone --branch curl-7_47_1 https://github.com/curl/curl.git
cd %IFM3D_BUILD_DIR%\curl
mkdir build
cd build
cmake -G %IFM3D_CMAKE_GENERATOR% -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config %CONFIG% --target INSTALL
```

### [xmlrpc-c](http://xmlrpc-c.sourceforge.net/)
```
cd %IFM3D_BUILD_DIR%
git clone --branch 1.33.14-cmake https://github.com/ifm/xmlrpc-c.git
cd %IFM3D_BUILD_DIR%\xmlrpc-c
mkdir build
cd build
cmake -G %IFM3D_CMAKE_GENERATOR% -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config %CONFIG% --target INSTALL
```

### [glog](https://github.com/google/glog)
```
cd %IFM3D_BUILD_DIR%
git clone --branch v0.3.5 https://github.com/google/glog.git
cd %IFM3D_BUILD_DIR%\glog
mkdir build
cd build
cmake -G %IFM3D_CMAKE_GENERATOR% -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config %CONFIG% --target INSTALL
```

### [gtest](https://github.com/google/googletest.git)
```
cd %IFM3D_BUILD_DIR%
git clone --branch release-1.8.1 https://github.com/google/googletest.git
```
NOTE: `gtest` is only needed to build and run unit tests. To skip, add
`-DBUILD_TESTS=OFF` to the cmake configuration command line on the `ifm3d`
library below.

# Building ifm3d
```
# Clone the repository
cd %IFM3D_BUILD_DIR%
git clone https://github.com/ifm/ifm3d.git
cd %IFM3D_BUILD_DIR%\ifm3d

# Configure
mkdir build
cd build
cmake -G %IFM3D_CMAKE_GENERATOR% -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON -DBUILD_SDK_PKG=ON -DGTEST_CMAKE_DIR=%IFM3D_BUILD_DIR%\googletest\googletest -Dgtest_force_shared_crt=TRUE -DCMAKE_PREFIX_PATH=%IFM3D_BUILD_DIR%\install;%IFM3D_OPENCV_PATH% -DBOOST_ROOT=%IFM3D_BOOST_ROOT% -DBoost_USE_STATIC_LIBS=ON -DCMAKE_BUILD_TYPE=%CONFIG% -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..

# run tests
cmake --build . --config %CONFIG% --target check

#install
cmake --build . --config %CONFIG% --target INSTALL
```

# Running the ifm3d command line tool
After Building `ifm3d`, the binary files will be installed at
``%IFM3D_BUILD_DIR%\install\bin``. To run the ifm3d tool you need to add this
directory to your path. You will also need to add the opencv directory to
your path.

If built targeting Visual Studio 2017/2019:
```
set PATH=%IFM3D_BUILD_DIR%\install\bin;%IFM3D_OPENCV_PATH%\x64\vc15\bin;%PATH%
```

If built targeting Visual Studio 2015:
```
set PATH=%IFM3D_BUILD_DIR%\install\bin;%IFM3D_OPENCV_PATH%\x64\vc14\bin;%PATH%
```

After that you should be able to run the ifm3d tool
```
ifm3d
```

# Appendix A: Building without PCL
The `ifm3d` library offers an alternative image buffer implementation which
only depends on `OpenCV`, thus eliminating a dependency on `PCL`. In order to
build `ifm3d` without a dependency on `PCL`, the following modifications to
the instructions above are necessary.

## Download Boost

`ifm3d` takes a dependency on the `Boost` libraries which are also distributed by `the `PCL all-in-one installer. Since we will not be installing `PCL`, `Boost` must be downloaded individually. Binaries for the latest release are available here (choose the version which matches your target msvc version):

* Visual Studio 2015: [boost_1_72_0-msvc-14.0-64.exe](https://sourceforge.net/projects/boost/files/boost-binaries/1.72.0/boost_1_72_0-msvc-14.0-64.exe/download)
* Visual Studio 2017: [boost_1_72_0-msvc-14.1-64.exe](https://sourceforge.net/projects/boost/files/boost-binaries/1.72.0/boost_1_72_0-msvc-14.1-64.exe/download)
* Visual Studio 2019: [boost_1_72_0-msvc-14.2-64.exe](https://sourceforge.net/projects/boost/files/boost-binaries/1.72.0/boost_1_72_0-msvc-14.2-64.exe/download)

## Set the proper BOOST_ROOT path

In the environment setup step, choose the proper path for BOOST_ROOT:
```
set IFM3D_BOOST_ROOT="C:\local\boost_1_72_0"
```

## Select the OpenCV Image Container

The `ifm3d::ImageBuffer` module (has dependency on PCL) must be disabled with
the flag `-DBUILD_MODULE_IMAGE=OFF` and the `ifm3d::OpenCV`Buffer module must
be enabled with the flag `-DBUILD_MODULE_OPENCV=ON`. The full `cmake`
configuration command is:

```
cmake -G %IFM3D_CMAKE_GENERATOR% -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON -DBUILD_SDK_PKG=ON -DGTEST_CMAKE_DIR=%IFM3D_BUILD_DIR%\googletest\googletest -Dgtest_force_shared_crt=TRUE -DCMAKE_PREFIX_PATH=%IFM3D_BUILD_DIR%\install;%IFM3D_OPENCV_PATH% -DBOOST_ROOT=%IFM3D_BOOST_ROOT% -DBoost_USE_STATIC_LIBS=ON -DCMAKE_BUILD_TYPE=%CONFIG% -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install -DBUILD_MODULE_IMAGE=OFF -DBUILD_MODULE_OPENCV=ON ..
```