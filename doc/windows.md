# How to build ifm3d library from source on Windows.

## Prepare dependent tools

Make sure that you have installed CMake and Visual Studio 64bit before you move to the next step.

* [CMake v3.9.1](http://www.cmake.org)
* [Visual Studio Community 2017](https://www.visualstudio.com)
* [Git for Windows](https://git-for-windows.github.io/) or any git client

Note: Visual Studio 2015 or later is required as older versions don't support all required features of C++11

## Download dependencies

Start with downloading the libraries from repositories listed below.

* [Glog](https://github.com/google/glog)
* [xmlrpc-c](https://github.com/graugans/xmlrpc-c)
* [curl](https://github.com/curl/curl)
* [Eigen](https://github.com/eigenteam/eigen-git-mirror)
* [FLANN](https://github.com/mariusmuja/flann)
* [VTK](https://gitlab.kitware.com/vtk/vtk)
* [PCL](https://github.com/PointCloudLibrary/pcl)



For all the download operations, the command line is a msys git bash. All operations are relative to each project top folder. 

### Downloading GLog
We recommend to download GLog by cloning the git repository.

```
$ git clone https://github.com/google/glog.git
$ cd glog
$ git checkout -b v0.3.5 v0.3.5
```
### Downloading xmlrpc-c
We recommend to download xmlrpc-c by cloning the git repository. For building the xmlrpc-c library with CMake we provide a CMake enabled version. Which should checkout the branch ``1.33.14-cmake`` by default.
Please ensure that you download the xmlrpc-c library into the 3rdParty folder of the ``ifm3d`` project.

```
$ cd 3rdParty
$ git clone https://github.com/graugans/xmlrpc-c.git
```

### Downloading curl

```
$ git clone https://github.com/curl/curl.git
$ git checkout -b curl-7_47_1 curl-7_47_1
```

### Download FLANN
```
$ git clone https://github.com/mariusmuja/flann.git
$ git checkout -b 1.9.1 1.9.1
```

### Download Eigen
```
$ git clone https://github.com/eigenteam/eigen-git-mirror.git
$ git checkout -b 3.3.4 3.3.4
```

### Download VTK
```
$ git clone https://gitlab.kitware.com/vtk/vtk.git
$ git checkout -b v6.3.0 -v6.3.0
```

### Download PCL
```
$ git clone https://github.com/PointCloudLibrary/pcl.git
$ git checkout -b pcl-1.8.1 pcl-1.8.1
```

### Download OpenCV
```
$ git clone https://github.com/opencv/opencv.git
$ git checkout -b 3.4.0 3.4.0
```

## Build the 3rdParty Libraries

The following command line examples assume we run from a windows command prompt with CMake in the Path variable.
Depending on your installation you may have to add CMake to your ``PATH`` variable.  Depending on 64 Bit or 32 Bit  installation of your system
your PATH variables may look different.

```
:: On a 64-bit machine running in 32-bit (WOW64) mode
set PATH=%PROGRAMW6432%\CMake\bin;%PATH%

:: On a 64-bit machine running in 64-bit
set PATH=%PROGRAMFILES%\CMake\bin;%PATH%
:: Or 
set PATH=%PROGRAMFILES(x86)%\CMake\bin;%PATH%
```

All the dependencies will by default get installed into "C:\ifm3d-deps\" please make sure you have write access to this folder or change the paths accordingly.

### Building curl

To build the curl library it is recommended to use CMake for setting the arch to x64.

```
> mkdir build
> cd build
> cmake -Ax64  -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON -DCMAKE_INSTALL_PREFIX=C:/ifm3d-deps/curl ..
> cmake --build . --clean-first --config Release --target INSTALL
```


### Building glog

Follow the build instructions in ``cmake/INSTALL.md`` for more details.

```
> mkdir build
> cd build
> cmake -A x64 -DCMAKE_INSTALL_PREFIX=C:/ifm3d-deps/glog ..
> cmake --build . --clean-first --config Release --target INSTALL
```


### Building FLANN
```
> mkdir build
> cd build
> cmake -A x64 -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_MATLAB_BINDINGS=OFF  -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_DOC=OFF -DCMAKE_INSTALL_PREFIX=C:/ifm3d-deps/flann ..
> cmake --build . --clean-first --config Release --target INSTALL
```


### Building Eigen
```
> mkdir build
> cd build
> cmake -A x64  -DCMAKE_INSTALL_PREFIX=C:/ifm3d-deps/eigen ..
> cmake --build . --clean-first --config Release --target INSTALL
```

### Building VTK
```
> mkdir build
> cd build
> cmake -A x64 -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=C:/ifm3d-deps/vtk ..
> cmake --build . --clean-first --config Release --target INSTALL
```

### Building PCL
```
> mkdir build
> cd build
> cmake -A x64 DPCL_ENABLE_SSE=OFF -DWITH_CUDA=OFF -DWITH_DAVIDSDK=OFF -DWITH_DOCS=OFF -DWITH_DSSDK=OFF -DWITH_ENSENSO=OFF -DWITH_FZAPI=OFF -DWITH_LIBUSB=OFF -DWITH_OPENGL=ON -DWITH_OPENNI=OFF -DWITH_OPENNI2=OFF -DWITH_PCAP=OFF -DWITH_PNG=OFF -DWITH_QHULL=OFF -DWITH_QT=OFF -DWITH_RSSDK=OFF  -DCMAKE_INSTALL_PREFIX=C:/ifm3d-deps/pcl ..
> cmake --build . --clean-first --config Release --target INSTALL
```

### Building OpenCV
```
> mkdir build_cmake
> cd build_cmake
> cmake -A x64 -DWITH_CUDA=OFF -DWITH_EIGEN=ON -DWITH_IPP=ON -DWITH_JASPTER=ON -DWITH_JPEG=ON -DWITH_OPENEXR=OFF -DWITH_OPENNI=OFF -DWITH_PNG=ON -DWITH_QT=OFF -DWITH_QT_OPENGL=OFF -DWITH_TBB=OFF -DWITH_TIFF_ON -DWITH_VIDEOINPUT=OFF -DBUILD_DOCS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_PACKAGE=OFF -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_openvc_python=OFF -DCMAKE_INSTALL_PREFIX=C:/ifm3d-deps/opencv ..
> cmake --build . --clean-first --config Release --target INSTALL
```


## Install dependencies

Install the necessary libraries on your system.

* [Boost v1.64.0](http://www.boost.org) Download [link](https://dl.bintray.com/boostorg/release/1.64.0/binaries/) to version 1.64 for Visual Studio 2017 (MSVC 14.1)

## Build the project via CMake.
```
> mkdir build
> cd build
> cmake -Ax64  -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON -DBUILD_SDK_PKG=OFF -DBUILD_TESTS=OFF -DBUILD_MODULE_EXAMPLES=OFF -DCMAKE_PREFIX_PATH="C:/ifm3d-deps/curl" -DBoost_INCLUDE_DIR="C:/ifm3d-deps/boost_1_64_0" -DBOOST_LIBRARYDIR="C:/ifm3d-deps/boost_1_64_0/lib64-msvc-14.1" -DBoost_COMPILER=-vc141 -DBoost_USE_STATIC_LIBS=ON -DEIGEN_INCLUDE_DIR="C:/ifm3d-deps/eigen/include/eigen3" -DCMAKE_BUILD_TYPE=Release ..
> cmake --build . --clean-first --config Release --target INSTALL
```

# Executing the ifm3d tools

After installation the PATH variable needs to be tweaked before we can use the ifm3d tools

```
> set PATH=%PROGRAMFILES(x86)%\CURL\bin;%PROGRAMFILES(x86)%\IFM3D\bin;%PROGRAMFILES(x86)%\IFM3D\lib;%PATH%
> ifm3d version
ifm3d: version=0.3.2
```

# TODO list

- [ ] Fix the CMake installation routines. On Windows not all artifacts are installed. For example the headers are missing
- [ ] Provide better support for xml-rpc
