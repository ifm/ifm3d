## Building ifm3d from source on Windows

This tutorial details how to compile the ifm3d library and its dependencies on
a Windows platform using Visual Studio.

### Dependencies

#### Build tools

* [CMake v3.11.0](http://www.cmake.org) or newer (also available through the
Visual Studio installer)
* [Git for Windows](https://gitforwindows.org) (also available through the
Visual Studio installer)
* [Microsoft Visual Studio](https://www.visualstudio.com)
version 2017 or 2019. The free 'Community' edition is sufficient. Be
sure to select the 'Desktop development with C++' workflow.

#### Source Dependencies

`ifm3d` depends on several additional libraries (curl, xmlrpc-c, glog, and
gtest) which are not available as binary packages on Windows.

##### Building source dependencies with ifm3d

ifm3d from version 0.90.4 onwards provides ```BUILD_IN_DEPS``` option to cmake configure command,
which fetches the required dependencies and builds them with ifm3d. After a successful first installation, the
user can disable BUILD_IN_DEPS option and can use the installed dependencies for future builds of the ifm3d.

Following instructions detail how to build ifm3d along with its dependencies.

⚠ The code on the branch {{ ifm3d_main_branch }} is updated nightly and contains the latest changes to the library. It is typically a work in progress.   
⚠ We recommend using tagged versions for your builds, to ensure consistency between builds. The latest tagged version can be found {{ '[here]({})'.format(ifm3d_latest_tag_url) }}.

Open Command Prompt and execute following instructions

```
#set the environment variables
set IFM3D_CMAKE_GENERATOR="Visual Studio 16 2019"
set IFM3D_BUILD_DIR=C:\ifm3d
set CONFIG=Release #set to Debug for debug binaries

#make the working dir
mkdir %IFM3D_BUILD_DIR%

# Clone the repository
cd %IFM3D_BUILD_DIR%
git clone https://github.com/ifm/ifm3d.git --branch o3r/main-next
cd %IFM3D_BUILD_DIR%\ifm3d

# Configure
mkdir build
cd build
cmake -G %IFM3D_CMAKE_GENERATOR% -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON -DBUILD_SDK_PKG=ON -DCMAKE_PREFIX_PATH=%IFM3D_BUILD_DIR%\install -DCMAKE_BUILD_TYPE=%CONFIG% -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install -DBUILD_IN_DEPS=ON ..

# Build ifm3d and dependencies
cmake --build . --config %CONFIG% --target ALL_BUILD

# install
cmake --build . --config %CONFIG% --target install
```
On successful execution of install step, user can disable the BUILD_IN_DEPS flag by appending
``` -DBUILD_IN_DEPS=OFF``` to cmake configure step, this will avoid building dependencies on every clean build.

### Running the ifm3d test with device (optional)

ifm3d tests are based on [gtest](https://github.com/google/googletest.git). Use following instructions to build and run the tests

```
# clone 
cd %IFM3D_BUILD_DIR%
git clone --branch release-1.8.1 https://github.com/google/googletest.git

cd ifm3d
cd build 
cmake -G %IFM3D_CMAKE_GENERATOR% -DCMAKE_WINDOWS_EXPORT_ALL_SYMBOLS=ON -DBUILD_SDK_PKG=ON -DBUILD_TESTS=ON -DGTEST_CMAKE_DIR=%IFM3D_BUILD_DIR%\googletest\googletest -Dgtest_force_shared_crt=TRUE -DCMAKE_PREFIX_PATH=%IFM3D_BUILD_DIR%\install;%IFM3D_OPENCV_PATH% -DCMAKE_BUILD_TYPE=%CONFIG% -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..

#Tests
cmake --build . --config %CONFIG% --target check
```

### Running the ifm3d command line tool
After Building `ifm3d`, the binary files will be installed at
``%IFM3D_BUILD_DIR%\install\bin``. To run the ifm3d tool you need to add this
directory to your path. You will also need to add the opencv directory to
your path.

If built targeting Visual Studio 2017/2019:
```
set PATH=%IFM3D_BUILD_DIR%\install\bin;%PATH%
```

After that you should be able to run the ifm3d tool
```
ifm3d
```

### How to install and use ifm3d binaries
1. Installation

    Download the ifm3d installer ifm3d_windows_x.x.x.exe.

    This installer provides binaries for Windows OS for Visual Studio 2019 and above. For any other compiler we recommend to build ifm3d from sources.

    Follow the instructions on the installer. It will install binaries at default location as C:/ProgramFiles/ifm3d x.x.x. Make a environment variable with this path.
    ```
    IFM3D_BINARY_DIR = C:/ProgramFiles/ifm3d x.x.x/bin      # Please put the correct install path in case installation is done on other than default path
    ```
2. Usage

    Clone ifm3d-example.

    Add install path (IFM3D_BINARY_DIR) to CMAKE_PREFIX_PATH and build ifm3d-example.