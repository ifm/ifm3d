

# How to build ifm3d library from source on Windows.

- [How to build ifm3d library from source on Windows.](#how-to-build-ifm3d-library-from-source-on-windows)
- [Preparations](#preparations)
    - [Prepare dependent tools](#prepare-dependent-tools)
    - [Getting the dependencies](#getting-the-dependencies)
        - [curl](#curl)
        - [xmlrpc-c](#xmlrpc-c)
        - [glog](#glog)
        - [FLANN](#flann)
        - [Eigen](#eigen)
        - [VTK](#vtk)
        - [PCL](#pcl)
        - [OpenCV](#opencv)
    - [Building ifm3d](#building-ifm3d)
- [Running ifm3d tool on Windows.](#running-ifm3d-tool-on-windows)



# Preparations
## Prepare dependent tools

Make sure that you have installed CMake and Visual Studio 64bit before you move to the next step.

* [CMake v3.9.1](http://www.cmake.org)
* [Visual Studio Community 2017](https://www.visualstudio.com)
* [Git for Windows](https://git-for-windows.github.io/) or any git client
* [Boost](http://www.boost.org) (Install
[1.64 for Visual Studio 2017 (MSVC 14.1)](https://dl.bintray.com/boostorg/release/1.64.0/binaries/boost_1_64_0-msvc-14.1-64.exe) or [1.64 for Visual Studio 2015 (MSVC 14.0)](https://dl.bintray.com/boostorg/release/1.64.0/binaries/boost_1_64_0-msvc-14.0-64.exe))


Note: Visual Studio 2015 or later is required as older versions don't support all required features of C++11

Note: Only x64 builds are suppported. 

The following command line examples assume we run from a windows command prompt with CMake and Git in the ``PATH`` variable.
Depending on your installation you may have to add CMake and Git to your ``PATH`` variable. 

First we need to set some variables, change these according to your installation.
```
set BOOST_INSTALL_DIR=C:\local\boost_1_64_0
set MSVC_MAJOR_VERSION=14
set MSVC_MINOR_VERSION=1
set IFM3D_BUILD_DIR=C:\ifm3d
```


## Getting the dependencies
Create the work directory
```
mkdir %IFM3D_BUILD_DIR%
```

Install all the dependencies

### [curl](https://curl.haxx.se/)
Download:
```
cd %IFM3D_BUILD_DIR%
git clone --branch curl-7_47_1 https://github.com/curl/curl.git
```

Build:
```
cd %IFM3D_BUILD_DIR%\curl
mkdir build
cd build
cmake -Ax64  -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config Release --target INSTALL
```

### [xmlrpc-c](http://xmlrpc-c.sourceforge.net/)
Download:
```
cd %IFM3D_BUILD_DIR%
git clone --branch 1.33.14-cmake https://github.com/graugans/xmlrpc-c.git
```

Build:
```
cd %IFM3D_BUILD_DIR%\xmlrpc-c
mkdir build
cd build
cmake -Ax64  -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config Release --target INSTALL
```

### [glog](https://github.com/google/glog)
Download:
```
cd %IFM3D_BUILD_DIR%
git clone --branch v0.3.5 https://github.com/google/glog.git
```

Build:
```
cd %IFM3D_BUILD_DIR%\glog
mkdir build
cd build
cmake -Ax64 -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config Release --target INSTALL
```

### [FLANN](https://www.cs.ubc.ca/research/flann/)
Download:
```
cd %IFM3D_BUILD_DIR%
git clone --branch 1.9.1 https://github.com/mariusmuja/flann.git
```

Build:
```
cd %IFM3D_BUILD_DIR%\flann
mkdir build
cd build
cmake -Ax64 -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF  -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_DOC=OFF -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config Release --target INSTALL
```

### [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
Download:
```
cd %IFM3D_BUILD_DIR%
git clone --branch 3.3.4 https://github.com/eigenteam/eigen-git-mirror.git eigen
```

Build:
```
cd %IFM3D_BUILD_DIR%\eigen
mkdir build
cd build
cmake -A x64 -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config Release --target INSTALL
```

### [VTK](https://www.vtk.org/)
Download:
```
cd %IFM3D_BUILD_DIR%
git clone --branch v6.3.0 https://gitlab.kitware.com/vtk/vtk.git
```

Build:
```
cd %IFM3D_BUILD_DIR%\vtk
mkdir build
cd build
cmake -A x64 -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config Release --target INSTALL
```

### [PCL](http://pointclouds.org/)
Download:
```
cd %IFM3D_BUILD_DIR%
git clone  --branch pcl-1.8.1 https://github.com/PointCloudLibrary/pcl.git
```

Build:
```
cd %IFM3D_BUILD_DIR%\pcl
mkdir build
cd build
cmake -A x64 DPCL_ENABLE_SSE=OFF -DWITH_CUDA=OFF -DWITH_DAVIDSDK=OFF -DWITH_DOCS=OFF -DWITH_DSSDK=OFF -DWITH_ENSENSO=OFF -DWITH_FZAPI=OFF -DWITH_LIBUSB=OFF -DWITH_OPENGL=ON -DWITH_OPENNI=OFF -DWITH_OPENNI2=OFF -DWITH_PCAP=OFF -DWITH_PNG=OFF -DWITH_QHULL=OFF -DWITH_QT=OFF -DWITH_RSSDK=OFF -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config Release --target INSTALL
```

### [OpenCV](https://opencv.org/)
Download:
```
cd %IFM3D_BUILD_DIR%
git clone  --branch 3.4.0 https://github.com/opencv/opencv.git
```

Build:
```
cd %IFM3D_BUILD_DIR%\opencv
mkdir build_cmake
cd build_cmake
cmake -A x64 -DWITH_CUDA=OFF -DWITH_EIGEN=ON -DWITH_IPP=ON -DWITH_JASPTER=ON -DWITH_JPEG=ON -DWITH_OPENEXR=OFF -DWITH_OPENNI=OFF -DWITH_PNG=ON -DWITH_QT=OFF -DWITH_QT_OPENGL=OFF -DWITH_TBB=OFF -DWITH_TIFF=ON -DWITH_VIDEOINPUT=OFF -DBUILD_DOCS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_PACKAGE=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_openvc_python=OFF -DCMAKE_PREFIX_PATH="%IFM3D_BUILD_DIR%\install"  -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config Release --target INSTALL
```

## Building ifm3d
Download:
```
cd %IFM3D_BUILD_DIR%
git clone  https://github.com/lovepark/ifm3d.git
```

Build
```
cd %IFM3D_BUILD_DIR%\ifm3d
mkdir build
cd build
cmake -Ax64 -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON -DBUILD_SDK_PKG=ON -DBUILD_TESTS=OFF -DBUILD_MODULE_EXAMPLES=ON -DCMAKE_PREFIX_PATH="%IFM3D_BUILD_DIR%\install" -DBoost_INCLUDE_DIR="%BOOST_INSTALL_DIR%" -DBOOST_LIBRARYDIR="%BOOST_INSTALL_DIR%/lib64-msvc-%MSVC_MAJOR_VERSION%.%MSVC_MINOR_VERSION%" -DBoost_COMPILER=-vc%MSVC_MAJOR_VERSION%%MSVC_MINOR_VERSION% -DBoost_USE_STATIC_LIBS=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config Release --target INSTALL
```


# Running ifm3d tool on Windows.
After Building ifm3d, the binary files will be installed at ``%IFM3D_BUILD_DIR%\install\bin``. To run the ifm3d tool you need to add this directory to your path. You will also need to add the opencv and boost binary directories to your ``PATH``.

For Visual Studio 2017 
```
Set PATH=%IFM3D_BUILD_DIR%\install\bin;%IFM3D_BUILD_DIR%\install\x64\vc15\bin;C:\local\boost_1_64_0\lib64-msvc-%MSVC_MAJOR_VERSION%.%MSVC_MINOR_VERSION%;%PATH%
```

For Visual Studio 2015 
```
Set PATH=%IFM3D_BUILD_DIR%\install\bin;%IFM3D_BUILD_DIR%\install\x64\vc%MSVC_MAJOR_VERSION%\bin;C:\local\boost_1_64_0\lib64-msvc-%MSVC_MAJOR_VERSION%.%MSVC_MINOR_VERSION%;%PATH%
```

After that you should be able to run the ifm3d tool
```
ifm3d 
```

