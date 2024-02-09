
## Building ifm3d from source on Windows

Following Instruction are for ifm3d-v1.0.0 and above which provide full support for O3R devices. if you are using O3D/O3X device please use ifm3d-v0.20.3
and follow these [instructions](https://github.com/ifm/ifm3d/blob/legacy/doc/windows.md)

This tutorial details how to compile the ifm3d library and its dependencies on
a Windows platform using Visual Studio and CMake.

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

`ifm3d` depends on several additional libraries (curl, xmlrpc-c, and
GoogleTest) which are not available as binary packages on Windows.

### Building source dependencies with ifm3d

ifm3d from version 0.90.4 onward provides ```BUILD_IN_DEPS``` option to CMake configure command,
which fetches the required dependencies and builds them with ifm3d. After a successful first installation, the
user can disable `BUILD_IN_DEPS` option and can use the installed dependencies for future builds of the ifm3d.

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
git clone https://github.com/ifm/ifm3d.git --branch v1.0.0
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
On successful execution of install step, user can disable the `BUILD_IN_DEPS` flag by appending
``` -DBUILD_IN_DEPS=OFF``` to CMake configure step, this will avoid building dependencies on every clean build.

### Building the Examples

To build the [examples](https://ifm3d.com/sphinx-doc/build/html/ifm3d/doc/sphinx/content/examples/index.html), provide the path `IFM3D_BINARY_DIR` to `CMAKE_PREFIX_PATH` when running CMake configure stage. 
To build the examples from source alongside the ifm3d library, enable the build with the `-DBUILD_EXAMPLES=ON`.

### Running ifm3d command line tools
After Building `ifm3d`, the binary files will be installed at
``%IFM3D_BUILD_DIR%\install\bin``. To run the ifm3d tool you need to add this
directory to your path.

If built targeting Visual Studio 2017/2019:
```bash
$ set PATH=%IFM3D_BUILD_DIR%\install\bin;%PATH%
```

After that you should be able to run the ifm3d tool
```bash 
$ ifm3d
```

### Using ifm3d-playground projects

After installing the ifm3d, one can use ifm3d-playground example, which shows basic CMake configuration required for using 
installed ifm3d libraries [ifm3d playground example](https://github.com/ifm/ifm3d/tree/main/examples/o3r/ifm3d_playground)