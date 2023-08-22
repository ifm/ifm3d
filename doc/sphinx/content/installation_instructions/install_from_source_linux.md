## Installing ifm3d from source

**Note**: Following Instruction are for ifm3d-v1.0.0 and above which provide full support for O3R devices. if you are using O3D/O3X device please use ifm3d-v0.20.3
and follow these [instructions](https://github.com/ifm/ifm3d/blob/v0.20.3/doc/source_build.md)

### Overview of the available build flags
| Flag name | Description | Default value |
| --------- | ----------- | ------------- |
| BUILD_MODULE_FRAMEGRABBER | Build the framegrabber module | ON | 
| BUILD_MODULE_TOOLS | Build the command-line utility | ON |  
| BUILD_IN_DEPS | Download and build dependencies | ON | 
| BUILD_MODULE_PYBIND11 | Build the ifm3dpy python package (it can also be installed directly through `pip`) | OFF | 
| USE_LEGACY_COORDINATES | Use the legacy coordinates (ifm3d <= 0.92.x) with swapped axis | OFF | 
| BUILD_MODULE_SWUPDATER | Build the swupdater module | ON | 
| BUILD_SDK_PKG | Build install packages for development purposes | ON | 
| BUILD_SHARED_LIBS | Build modules as shared libraries | ON | 
| BUILD_EXAMPLES | Build the examples | OFF | 
| BUILD_DOC | Build documentation | OFF |
| BUILD_MODULE_PCICCLIENT | Build the pcicclient module | OFF |
| BUILD_IN_DEPS | Download, build and install required dependencies with ifm3d (for ifm3d v0.93.0 and above) | ON |
### Build Dependencies

<table>
  <tr>
    <th>Dependency</th>
    <th>Dependent ifm3d module</th>
    <th>Notes</th>
  </tr>
  <tr>
    <td><a href="http://www.cmake.org">CMake</a></td>
    <td>device, framegrabber, swupdater, pcicclient, tools, pybind11</td>
    <td>Meta-build framework</td>
  </tr>
  <tr>
    <td><a href="https://curl.haxx.se/libcurl">Curl</a></td>
    <td>device, tools, swupdater</td>
    <td>Used to help enable command-line based firmware flashing.</td>
  </tr>
  <tr>
    <td><a href="https://github.com/google/glog">Glog</a></td>
    <td>device, framegrabber, swupdater, pcicclient, tools, pybind11</td>
    <td>Logging framework</td>
  </tr>
  <tr>
    <td><a href="https://github.com/google/googletest">Gtest</a></td>
    <td>device, framegrabber, swupdater, pcicclient, tools, pybind11</td>
    <td>Unit testing framework</td>
  </tr>
  <tr>
    <td><a href="http://xmlrpc-c.sourceforge.net/">libxmlrpc</a></td>
    <td>device, pybind11</td>
    <td>XMLRPC client used call into the camera configuration interface</td>
  </tr>
  <tr>
    <td><a href="https://github.com/pybind/pybind11">pybind11</a></td>
    <td>pybind11</td>
    <td>A header-only library that exposes C++ types in Python and vice versa,
    mainly to create Python bindings of existing C++ code.</td>
  </tr>
</table>


### Building From Source
Start with cloning the code from the ifm3d github repository {{ '[here]({})'.format(ifm3d_gh_url) }}.

⚠ The code on the branch {{ ifm3d_main_branch }} is updated nightly and contains the latest changes to the library. It is typically a work in progress.  
⚠ We recommend using tagged versions for your builds, to ensure consistency between builds. The latest tagged version can be found {{ '[here]({})'.format(ifm3d_latest_tag_url) }}.

#### The default build

By default, the `ifm3d` build enables the `device`, `framegrabber`, `swupdater`,
and `tools` modules. Building the software follows the usual cmake idiom of:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr ..
$ cmake --build .
$ sudo cmake --build . --target install
```

This will build and install ifm3d along with its dependencies.

> Note: you can speed up the build by using `ninja`, with `cmake -GNinja -DCMAKE_INSTALL_PREFIX=/usr ..`.


#### Building the examples

The examples can be built along with the rest of the library by switching the DBUILD_EXAMPLES flag on. Assuming you are in the /build folder:

```
$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_EXAMPLES=ON ..
$ cmake --build . --target ALL_BUILD
```

### Build debian packages from source

This additional section provides instructions to build Debian packages from ifm3d source.  
To install the existing ifm3d Debian packages please refer to [this](ifm3d/doc/sphinx/content/installation_instructions/install_linux_binary:Installing%20ifm3d%20from%20.deb%20file) section.

#### Install dependencies for ifm3d debian packages

If you plan to build the debian packages and have the
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
      libssl-dev \
      libcurl4-openssl-dev \
      libgtest-dev libgoogle-glog-dev  \
      libxmlrpc-c++8-dev \ 
      libproj-dev \
      build-essential \
      coreutils \
      cmake

# Only if you wish to build the python bindings
$ sudo apt-get update && sudo apt-get install pybind11-dev                          
```
Note: The package name may differ in different flavours of Linux. 
Above apt-get commands are specific to Debian based systems


#### Building debian packages

Alternatively, to build *debs* to be distributed to multiple runtime machines, you can use the following:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr ..
$ cmake --build .
$ cmake --build . --target package
$ cmake --build . --target repackage

$ sudo dpkg -i ifm3d_*_amd64-common.deb
$ sudo dpkg -i ifm3d_*_amd64-deserialize.deb
$ sudo dpkg -i ifm3d_*_amd64-device.deb
$ sudo dpkg -i ifm3d_*_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_*_amd64-swupdater.deb
$ sudo dpkg -i ifm3d_*_amd64-tools.deb
```

(The version number embedded in the deb file will be dependent upon which
version of the `ifm3d` software you are building)

> Note: Experienced users may be puzzled by the `repackage` step. 
> This step is used to dynamically compute the debian dependencies for the particular module. 
> Due to how we are partitioning out the software, this approach is necessary vs. the more traditional `CPACK_DEBIAN_PACKAGE_SHLIBDEPS` wrapper around `dpkg-shlibdeps`. 
> We basically created [a version of that tool](cmake/utils/ifm3d-dpkg-deps.py.in) that exploits *a-priori* information about the `ifm3d` environment to properly compute the debian dependencies. 
> If you are building debs on a build machine to be distributed out to various runtime computers, you will certainly want to execute the `repackage` target so that you are ensured the runtime machines have the proper dependency chain in place.
