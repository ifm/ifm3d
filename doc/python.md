Python bindings for ifm3d
-------------------------

The `pybind11` module provides a direct Python binding to the `ifm3d` C++ APIs,
built with the [pybind11](https://github.com/pybind/pybind11) library.

All generic functionality from the `camera` and `framegrabber` modules
are exposed as Python APIs. Additionally, a custom `ImageBuffer` implementation
exposes frame data to Python as `numpy` arrays in a zero copy manner.

> :warning: Support for python 2.x has been deprecated in accordance with the
> general python 2 end of life. These instructions may work for python 2 but
> are not officially supported or tested.

## Python Dependencies

The `pybind11` module leverages `numpy` as an image container. Ensure `numpy`
has been installed in your python environment:

```
$ python -m pip install numpy
```

## Linux Installation

The `pybind11` module can be built in several ways depending on your python
workflow.

* apt repository: Recommended for system-wide installations on
  supported Ubuntu platforms
* setuptools: Recommended for most other installations,
  especially sandboxed environments.
* CMake: Recommended for customized installations where shared
  libraries are desired.

### From apt

The ifm apt server for Ubuntu LTS releases includes packages for the
distribution provided version of python3 via the `ifm3d-python3` package.

Follow the instructions on the [README](../README.md) to configure the
appropriate apt repository for your environment, and then install the packages
of choice.

### Building from source (CMake or setuptools)

The `pybind11` module can be built and managed as a standard shared library
(recommended for installations targeting system-level python interpreters) or
as a standalone setuptools project (recommended for virtualenv, conda, etc).

#### Build Dependencies (same for CMake and setuptools)

First, satisfy all dependencies for the `ifm3d-camera` and
`ifm3d-framegrabber` modules according to the instructions on [Installing
ifm3d from source](source_build.md).

Next, the [pybind11](https://github.com/pybind/pybind11) project must be
installed and accessible to CMake. The easiest way to do this is to clone and
install (the latest tested version is v2.3.0, but newer releases ought to work
as well):

```
$ git clone https://github.com/pybind/pybind11
$ cd pybind11
$ git checkout tags/v2.3.0
$ mkdir build
$ cd build
$ cmake -DPYBIND11_TEST=OFF ..
$ make
$ sudo make install
```

Finally, ensure the *-dev packages have been installed for your python
interpreter of choice. For example, to target the Ubuntu python interpreter:

```
$ sudo apt-get install python3-dev
```

#### Compiling and Installing with setuptools

For users where the apt repository does not apply, or users of sandboxed
python installations (including virtualenv, conda, etc) we provide
a setuptools compliant `setup.py` in the top level of the ifm3d project.

One important distinction with the setuptools install as compared with the
CMake/apt/debian installation is that the `ifm3d-camera` and
`ifm3d-framegrabber` modules are built with static linkage. This allows
workflows such as sandboxing different ifm3d versions in different
virtualenvs, or allowing a python module to operate independently with other
system-wide `ifm3d` packages. Furthermore, the `ifm3d-camera` and
`ifm3d-framegrabber` modules do not need to be built and installed as packages
on the system.

To use, simply satisfy the requirements and execute setup.py directly, through
pip, etc.

An example of how to install to an anaconda environment follows:

```
$ conda activate base
(base)$ cd ifm3d
(base)$ pip install .
```

And `pip show -f ifm3dpy` produces the following:

```
Name: ifm3dpy
Version: 0.18.0
Summary: Library for working with ifm pmd-based 3D ToF Cameras
Home-page: https://github.com/ifm/ifm3d
Author: Sean Kelly
Author-email: Sean.Kelly@ifm.com
License: Apache 2.0
Location: <path/to/anaconda3>/lib/python3.7/site-packages
Requires:
Required-by:
Files:
  ifm3dpy-0.18.0.dist-info/INSTALLER
  ifm3dpy-0.18.0.dist-info/LICENSE
  ifm3dpy-0.18.0.dist-info/LICENSE.MIT
  ifm3dpy-0.18.0.dist-info/METADATA
  ifm3dpy-0.18.0.dist-info/RECORD
  ifm3dpy-0.18.0.dist-info/WHEEL
  ifm3dpy-0.18.0.dist-info/top_level.txt
  ifm3dpy.cpython-37m-x86_64-linux-gnu.so
```

#### Compiling and Installing with CMake

The `pybind11` module may be compiled and installed from source via CMake
(same semantics as the rest of the ifm3d modules) by specifying
`-DBUILD_MODULE_PYBIND11=ON` on your CMake command.

The [pybind11](https://github.com/pybind/pybind11) library can be made to
target different python versions. By default, it will use the interpreter to
which `python` resolves in your environment. To specify an alternative
interpreter, use the `-DPYTHON_EXECUTABLE=/path/to/python` flag in your CMake
command.

As an example, the commands to build the default + `pybind11` modules are:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_MODULE_PYBIND11=ON \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 ..
$ make
```

Once compiled, installation can either be performed via `sudo make install`,
or for debian-based systems, through dpkg:

```
$ make package
$ make repackage
$ sudo dpkg -i ifm3d-camera.deb \
               ifm3d-framegrabber.deb \
               ifm3d-python3.deb
```

**NOTE:** The CMake scripts query the selected python interpreter for its
relative packages directory (either `site-packages` or `dist-packages`). The
CMake install/packaging scripts will install the resulting library to
`${CMAKE_INSTALL_PREFIX}/<path/to/dist-packages>`, which only makes sense for
system-wide python installations. For non-system python installations
(virtualenv, conda, etc) we strongly recommend installing through
`setuptools`. If you must build and install via CMake and shared libraries,
you must ensure the resulting .so is found on your `$PYTHONPATH`.
Alternatively, the python installation directory can be overridden by adding
`-DPYTHON_PACKAGES_PATH=/path/to/your/interpreter/libs` to the CMake command.

## Windows Installation

For Windows installations, as there is no default system-wide python out of the
box, we recommend using `pip` and `setuptools` to minimize complexity.

### Build Dependences

First, follow all the setup and dependency installation steps found in the
[Windows Installation](windows.md) documentation. For bare minimum python
requirements, follow the guidance for the `OpenCV` image container instead of
the `Image` image container. In other words, the only external lib requirements
are `boost`, `curl`, `xmlrpc-c`, `glog` and `OpenCV`. For now, **only** install
dependencies; do not continue on to installing ifm3d.

The `pybind11` module takes one additional dependency:

#### [Pybind11](https://github.com/pybind/pybind11)
Download:
```
cd %IFM3D_BUILD_DIR%
git clone https://github.com/pybind/pybind11.git
cd pybind11
git checkout tags/v2.3.0
```

Build:
```
mkdir build
cd build
cmake -G %IFM3D_CMAKE_GENERATOR% -DPYBIND11_TEST=OFF -DCMAKE_INSTALL_PREFIX=%IFM3D_BUILD_DIR%\install ..
cmake --build . --clean-first --config %CONFIG% --target INSTALL
```

### Building

First, clone the ifm3d repo:
```
cd %IFM3D_BUILD_DIR%
git clone  https://github.com/ifm/ifm3d.git
```

*Note*: At this point, you may continue on to follow the steps under 'Building
ifm3d' in the  [Windows Installation](windows.md) guide, but it is not
necessary for the python bindings. Pip will build and install from a separate
temporary directory.

Next, follow the appropriate guidance under 'Running ifm3d tool on Windows' to
populate the `PATH` variable according to the version of Visual Studio you are
using.

Finally, with the dependencies installed and on the system path, you are ready
to let `setup.py` compile and install the python binary:

```
cd %IFM3D_BUILD_DIR%\ifm3d
python -m pip install .
```

## Running Tests

The `pybind11` module ships with pytest unit tests, so ensure pytest is
installed:

```
$ python -m pip install pytest
```

The tests can be executed as follows:

```
$ cd modules/pybind11/test
$ python -m pytest .
```

## Usage

The semantics of the python library are intentionally a 1:1 mapping with the
C++ API, so all samples and existing code can port directly. Here is a python
translation of the C++ sample found in the [ifm3d Basic Library
Usage](basic_usage.md) document.

```python
#!/usr/bin/env python
import sys
import ifm3dpy

def main():
    cam = ifm3dpy.Camera()
    fg = ifm3dpy.FrameGrabber(cam)
    im = ifm3dpy.ImageBuffer()

    while True:
        if not fg.wait_for_frame(im, 1000):
            print("Timeout waiting for camera!")
            return -1

        amp = im.amplitude_image()
        xyz = im.xyz_image()

        # now do something with `amp` and `xyz`,
        # which are of type numpy.ndarray

if __name__=='__main__':
    sys.exit(main())
```

There are a couple aspects of the python bindings that are worth highlighting.

### Memory Ownership

In order to maximize performance, memory is allocated and owned in the C++
layer that backs the python library. Pixel data is *not* copied across. So
while `amp` and `xyz` are `numpy.ndarray` objects in the above sample, they do
not own their buffers (to prove this, see that `xyz.flags['OWNDATA']` is set
to `False`).

One practical implication is that in the above sample, subsequent calls to
`fg.wait_for_frame(im, 1000)` will override the image data represented in the
`amp` and `xyz` objects. This is not dissimilar from how the C++ API operates,
but may be surprising to python users familiar with 'standard' numpy arrays
which own their own memory.

If this behavior is not desired, one may simply copy the `numpy` array prior
to capturing subsequent images.

Finally, the C++ binding library will take care of releasing memory when all
relevant python objects have gone out of scope. Furthermore, the C++ binding
library captures the memory of each image within a capsule, so even if the
underlying `ifm3dpy.FrameGrabber` object goes out of scope, memory for pixel
data will remain valid as long as a referencing python object remains in
scope.

### Multithreading and Multiprocessing

Due to the single ASIO event loop under the hood, *multiprocessing* is not
supported by this library and will lead to unwanted behavior.

The C++ bindings do support true multithreading and release the python GIL for
long running operations. This is most useful for scenarios where one thread is
blocking on `wait_for_frame` and another thread is software triggering.

### ifm3dpy.FrameGrabber.reset() Method

One usage pattern of the ifm3d C++ API is to use the
`std::shared_ptr<ifm3d::FrameGrabber>::reset()` method in order to change
associated cameras or imager mask settings. Since this happens directly at the
`std::shared_ptr` level, a helper method `ifm3dpy.FrameGrabber.reset()` has
been created to emulate this usage pattern in Python. The following example
demonstrates caching the unit vectors, changing the `FrameGrabber` schema, and
then streaming data.

```python
#!/usr/bin/env python
import ifm3dpy

cam = ifm3dpy.Camera()
fg = ifm3dpy.FrameGrabber(cam, ifm3dpy.IMG_UVEC)
im = ifm3dpy.ImageBuffer()

# Grab the unit vectors
fg.wait_for_frame(im, 1000)
uvec = im.unit_vectors()

# Change the schema to radial distance
fg.reset(cam, ifm3dpy.IMG_RDIS)
fg.wait_for_frame(im, 1000)
rdis = im.distance_image()
```

