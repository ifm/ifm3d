## Installing ifm3d from source

**Note**: Following Instruction are for ifm3d-v1.0.0 and above which provide full support for O3R devices. if you are using O3D/O3X device please use ifm3d-v0.20.3
and follow these [instructions](https://github.com/ifm/ifm3d/blob/v0.20.3/doc/source_build.md)

### Overview of the available build flags
| Flag name | Description | Default value |
| --------- | ----------- | ------------- |
| `BUILD_MODULE_FRAMEGRABBER` | Build the `framegrabber` module | ON | 
| `BUILD_MODULE_TOOLS` | Build the command-line utility | ON |  
| `BUILD_MODULE_DESERIALIZE` | Build the `deserialize` module | ON | 
| `BUILD_MODULE_PYBIND11` | Build the ifm3dpy Python package (it can also be installed directly through `pip`) | OFF | 
| `USE_LEGACY_COORDINATES` | Use the legacy coordinates (ifm3d <= 0.92.x) with swapped axis | OFF | 
| `BUILD_MODULE_SWUPDATER` | Build the `swupdater` module | ON | 
| `BUILD_SDK_PKG` | Build install packages for development purposes | ON | 
| `BUILD_SHARED_LIBS` | Build modules as shared libraries | ON | 
| `BUILD_EXAMPLES` | Build the examples | OFF | 
| `BUILD_DOC` | Build documentation | OFF |
| `BUILD_MODULE_PCICCLIENT` | Build the `pcicclient` module | OFF |
| `BUILD_IN_DEPS` | Download, build and install required dependencies with ifm3d (for ifm3d v0.93.0 and above) | ON |

### Build Dependencies

The ifm3d library depends on the libraries listed below. Only CMake and pybind11 (if building the Python library) need to be installed on the user's machine. The other dependencies will be pulled automatically. 

| Dependency | Dependent ifm3d module | Notes |
|:---------- |:---------------------- |:----- |
| CMake| `device`, `framegrabber`, `swupdater`, `pcicclient`, `tools`, `pybind11`| Meta-build framework|
| curl| `device`, `tools`, `swupdater`| Used to help enable command-line based firmware flashing.|
| GoogleTest| `device`, `framegrabber`, `swupdater`, `pcicclient`, `tools`, `pybind11`| Unit testing framework|
| libxmlrpc| `device`, pybind11| XMLRPC client used call into the camera configuration interface|
| pybind11| `pybind11`| A header-only library that exposes C++ types in Python and vice versa,  mainly to create Python bindings of existing C++ code.|

### Building From Source
Start with cloning the code from the ifm3d GitHub repository {{ '[here]({})'.format(ifm3d_gh_url) }}.

⚠ The code on the branch {{ ifm3d_main_branch }} is updated nightly and contains the latest changes to the library. It is typically a work in progress.  
⚠ We recommend using tagged versions for your builds, to ensure consistency between builds. The latest tagged version can be found {{ '[here]({})'.format(ifm3d_latest_tag_url) }}.

#### The default build

By default, the `ifm3d` build enables the `device`, `framegrabber`, `deserializer`, `swupdater`,
and `tools` modules. Building the software follows the usual CMake idiom of:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr ..
$ cmake --build .
$ sudo cmake --build . --target install
```

This will build and install ifm3d along with its dependencies.

> Note: you can speed up the build by using `ninja`, with `cmake -GNinja -DCMAKE_INSTALL_PREFIX=/usr ..`.

### Build Debian packages from source

#### Dependencies
If you plan to build the Debian packages and have the
dependencies computed for you dynamically (see the note below on the
`repackage` target), you will also need:

* [readelf](https://www.gnu.org/software/binutils/) (Part of the `binutils` package)
* [ldd](http://man7.org/linux/man-pages/man1/ldd.1.html) (Part of the `libc-bin` package)
* [dpkg](https://help.ubuntu.com/lts/serverguide/dpkg.html)

We note that, if you are running on a supported Linux, all of these packages
are available through the official Debian repositories and should be a simple
`apt-get` away from being installed on your machine.

This additional section provides instructions to build Debian packages from ifm3d source.  
To install the existing ifm3d Debian packages please refer to [this](ifm3d/doc/sphinx/content/installation_instructions/install_linux_binary:Installing%20ifm3d%20from%20.deb%20file) section.

#### Building Debian packages

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
> This step is used to dynamically compute the Debian dependencies for the particular module. 
> Due to how we are partitioning out the software, this approach is necessary vs. the more traditional `CPACK_DEBIAN_PACKAGE_SHLIBDEPS` wrapper around `dpkg-shlibdeps`. 
> We basically created [a version of that tool](cmake/utils/ifm3d-dpkg-deps.py.in) that exploits *a-priori* information about the `ifm3d` environment to properly compute the Debian dependencies. 
> If you are building debs on a build machine to be distributed out to various runtime computers, you will certainly want to execute the `repackage` target so that you are ensured the runtime machines have the proper dependency chain in place.
