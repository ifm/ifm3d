## Installing ifm3d from source

**Note**: Following Instruction are for ifm3d-v1.6.0 and above which provide full support for O3R, O3D and O3X devices.

### Overview of the available build flags
| Flag name | Description | Default value |
| --------- | ----------- | ------------- |
| `BUILD_MODULE_FRAMEGRABBER` | Build the `framegrabber` module | ON | 
| `BUILD_MODULE_TOOLS` | Build the command-line utility | ON |  
| `BUILD_MODULE_DESERIALIZE` | Build the `deserialize` module | ON | 
| `BUILD_MODULE_PYBIND11` | Build the ifm3dpy Python package (it can also be installed directly through `pip`) | OFF | 
| `BUILD_MODULE_SWUPDATER` | Build the `swupdater` module | ON | 
| `BUILD_MODULE_PCICCLIENT` | Build the `pcicclient` module | OFF |
| `BUILD_SDK_PKG` | Build install packages for development purposes | ON | 
| `BUILD_SHARED_LIBS` | Build modules as shared libraries | ON | 
| `BUILD_EXAMPLES` | Build the examples | OFF | 
| `BUILD_DOC` | Build documentation | OFF |

### Build Dependencies

The ifm3d library depends on the libraries listed below. Only CMake and pybind11 (if building the Python library) need to be installed on the user's machine. The other dependencies will be pulled automatically. 

| Dependency | Dependent ifm3d module | Notes |
|:---------- |:---------------------- |:----- |
| CMake| `device`, `framegrabber`, `swupdater`, `pcicclient`, `tools`, `pybind11`| Meta-build framework|

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

To build *debs* to be distributed to multiple runtime machines, you can use the following:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr ..
$ cmake --build .
$ cmake --build . --target package

$ sudo dpkg -i ifm3d_*_amd64.deb
```

(The version number embedded in the deb file will be dependent upon which
version of the `ifm3d` software you are building)

