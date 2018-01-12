# How to build ifm3d library from source on Windows.

As a first step we build ifm3d without the image module.

## Prepare dependent tools

Make sure that you have installed CMake and VS2017 64bit before you move to the next step.

* [CMake v3.9.1](http://www.cmake.org)
* [Visual Studio Community 2017](https://www.visualstudio.com)
* [Git for Windows](https://git-for-windows.github.io/) or any git client

## Download dependencies

Start with downloading Glog, curl and XMLRPC libraries from repository listed below.

* [Glog](https://github.com/google/glog)
* [xmlrpc-c](https://github.com/graugans/xmlrpc-c)
* [curl](https://github.com/curl/curl)

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

### Building curl

To build the curl library it is recommended to use CMake for setting the arch to x64.

```
> mkdir build
> cd build
> cmake -Ax64  -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON ..
> cmake --build . --clean-first --config Release --target INSTALL
```
After a successful curl build the build artifacts will be for example found in ``%PROGRAMFILES(x86)%\CURL``. Please double check this with your configuration and platform.


### Building glog

Follow the build instructions in ``cmake/INSTALL.md`` for more details.

```
> mkdir build
> cd build
> cmake -A x64 ..
> cmake --build . --clean-first --config Release --target INSTALL
```


## Install dependencies

Install the necessary libraries on your system.

* [Boost v1.64.0](http://www.boost.org) Download [link](https://dl.bintray.com/boostorg/release/1.64.0/binaries/) to version 1.64 for Visual Studio 2017 (MSVC 14.1)

## Build the project via CMake.

Due to the fact there are no default locations under Windows for include files and libraries we have to provide CMake some hints.

```
> mkdir build
> cd build
> :: The CMAKE_PREFIX_PATH depends on the location you previously installed CURL to
> set CMAKE_PREFIX_PATH=%PROGRAMFILES(x86)%\CURL
> cmake -Ax64  -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON -DBUILD_SDK_PKG=OFF -DBUILD_TESTS=OFF -DBUILD_MODULE_IMAGE=OFF -DBUILD_MODULE_EXAMPLES=OFF -DBoost_INCLUDE_DIR="D:\local\boost_1_64_0" -DBOOST_LIBRARYDIR=D:\local\boost_1_64_0\lib64-msvc-14.1 -DBoost_COMPILER=-vc141 -DBoost_USE_STATIC_LIBS=ON  -DCMAKE_BUILD_TYPE=Release ..
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

- [ ] Update the documentation to include the ``image`` module
- [ ] Fix the CMake installation routines. On Windows not all artifacts are installed. For example the headers are missing
- [ ] Provide better support for xml-rpc
