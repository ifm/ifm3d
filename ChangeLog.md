## Changes between ifm3d 0.17.0 and 0.18.0
* Support for latest O3D3XX firmware (1.30.5309)
* Support for latest O3X firmware (1.0.156)
* Deprecated ROS-specific apt repositories
* Deprecated python2 support
* Support for Ubuntu 20.04 Focal Fossa
* Packaged and released ifm3d as a Snap
* Added support for Ubuntu ARM64
* Added support for L4T (Linux for Tegra) JetPack 4.3 and 4.4
* Improved Windows build instructions
* Created GitHub Actions CI workflows
* Bugfixes:
  * #190 - Added missing include for Windows build

## Changes between ifm3d 0.16.0 and 0.17.0
* Reverted changes in 0.16.0 (FrameGrabberUdp -- No viable path to UDP
  implementation in F/W)
* Bugfixes
  * Issue with libcurl usage on 32bit targets
  * Corrected minimum firmware version required for inverse intrinsics
  * Corrected handling of spurious wakes in FrameGrabber
  * Fixed ComputeCartesian python unit test to properly blank out invalid
    pixels
  * Changed `build` Dockerfiles to use pip for numpy/pytest
  * Fixed race condition in PCICClient unit tests
  * Changed setup.py to honor the environment variables per the Windows installation instructions
  * Updated installation documentation for Windows

## Changes between ifm3d 0.15.1 and 0.16.0
* Created new `framegrabberudp` module for consuming data over UDP interface

## Changes between ifm3d 0.15.0 and 0.15.1
* Minor updates to allow for cross-compiling ifm3d for the O3D3XX
*  PCIC timeout issue fixed

## Changes between ifm3d 0.14.1 and 0.15.0
* Added Interface for getting json_model from O3D3xx devices.

## Changes between ifm3d 0.14.0 and 0.14.1
* Fixes to how timeouts are handled in `swupdate` module
* Updated embedded JSON library to
  [3.6.1](https://github.com/nlohmann/json/releases/tag/v3.6.1),
  single-header.

## Changes between ifm3d 0.13.0 and 0.14.0
* New module: swupdater -- utilities for updating camera firmware
    * Ported functionality from `swupdate` command into its own library for
      programmatic consumption.
    * Updated certain semantics of the `swupdate` command in the `tools`
      module to match those of the other ifm3d `tools` commands
        * Updated command line switch naming to match other ifm3d tools:
          * `check` subcommand now invoked by `-c` or `--check`
          * `reboot` subcommand now invoked by `-r` or `--reboot`
        * `file` subcommand will now test for recovery and automatically
          reboot the device into recovery as needed.
* Disabled framegrabber's InverseIntrinsicParamSchema test due to suspected
  false failures. Test case will be investigated and re-opened in a future
  release.
* Fixed issues with unit test scripts on Windows
* Fixed Windows build documentation
  * Added `BUILD_SHARED_LIBS` definition to `glog` to address issues with
    logging to STDERR in Windows binaries
  * Parameterized the CMake generator for easier building when multiple
    versions of MSVC are installed concurrently

## Changes between ifm3d 0.12.0 and 0.13.0
* Honor semantics of CMake's BUILD_SHARED_LIBS flag (ON by default). Setting
  to off will build and link against ifm3d modules as static libraries.
* New module: pybind11 -- Python bindings for the the C++ API

## Changes between ifm3d 0.11.2 and 0.12.0

* Fixes to build infrastructure in support of windows unit tests
* Added support to retrieve the inverse intrinsic parameters from O3D3xx
  cameras

## Changes between ifm3d 0.11.1 and 0.11.2

* Bugfix for #111, moved a log message in framegrabber to IFM3D_PROTO_DEBUG to
  keep noise level low when running an O3X for extended periods of time.
* Changed flagging bad pixels to always be `0` regardless of data type. Users
  could always consult the confidence image themselves and discriminate between
  a true `0` (not possible) and a bad pixel which they could then transform to
  `nan` or whatever other sentinel makes sense for their application.

## Changes between ifm3d 0.11.0 and 0.11.1

* Bugfix for #103 ``header is not in the correct format`` when ``make check`` is
  executed against FW 1.6.2114
* Bugifx for #107 Allows OpenCV module headers to be included in more than one
  translation unit thus avoiding violation of ODR.
* The `image` and `opencv` modules now flags bad pixels at the driver-level

## Changes between ifm3d 0.10.0 and 0.11.0

* Added a `jitter` subcommand to ifm3d
* Added support to retrieve the intrinsic parameters from O3D3xx cameras

## Changes between ifm3d 0.9.3 and 0.10.0

* Adds support for setting the `IFM3D_SESSION_ID` environment variable for
  establishing edit sessions with the camera using a known ID.
* Sessions are now explicitly cancellable if the session ID is known.
* Some session management optimizations in `FromJSON` which should result in
  incremental speedups in importing JSON configurations to the camera.


## Changes between ifm3d 0.9.2 and 0.9.3

* Added build instructions how to switch between Release and Debug
  for Windows builds
* Added Troubleshoot guide
* Added Opencv module build instruction for windows
* Added minimum MSVC version requirement
* Added prerequisite packages list for building ifm3d
* Changed warning message in framegrabber from `WARNING` to `IFM3D_TRACE`
  severity level
* Updated swupdate command with --check (recovery mode check) and -r (reboot to
  productive mode)

## Changes between ifm3d 0.9.1 and 0.9.2

* Added Support for the Ubuntu 18.04
* Added glog support in the cmake config files

## Changes between ifm3d 0.9.0 and 0.9.1

* Removed some additional Boost dependencies

## Changes between ifm3d 0.8.3 and 0.9.0

* Version number handling is now done in the cmake `project` command in the
  top-level CMakeLists.txt file
* Dropped support for Ubuntu 14.04
* Increased cmake requirements to 3.5
* Increaded compiler requirements to C++14
* Moved `examples` module into new project
  [ifm3d-examples](https://github.com/lovepark/ifm3d-examples)
* Moved `viewer` sub-command out of the ifm3d project. This is to decrease the
  PCL dependencies (see Issue #42). A new project will be created in support of
  this viewer application:
  [ifm3d-pcl-viewer](https://github.com/lovepark/ifm3d-pcl-viewer)
* Updated JSON parsing library to 3.1.2
* By default, pcicclient module is now `OFF`.
* Pixel-parsing framework has been significantly refactored. Sub-system
  specific docs for image container implementers have been provided in the
  `doc` folder.
* Updated the `ImageBuffer` to conform to the new pixel-publishing
  architecture.
* Initial implementation of an OpenCV-only (i.e., no PCL) image container. This
  is the `opencv` module of the `ifm3d` project.
* Added a `passwd` subcommand to `ifm3d`

## Changes between ifm3d 0.8.2 and 0.8.3

* Fixed a cmake regression regarding -std=c++11 flags passed to the compiler;
  surfaces on old versions of cmake, i.e., in Ubuntu 14.04

## Changes between ifm3d 0.8.1 and 0.8.2

* Patch to windows build
* Better semver parsing of camera firmware

## Changes between ifm3d 0.8.0 and 0.8.1

* Reverted Windows build changes due to how it broke packaging on Linux

## Changes between ifm3d 0.7.0 and 0.8.0

* Illumination temperature is registered to frame data

## Changes between ifm3d 0.6.0 and 0.7.0

* Added timestamping of image buffers
* Added support for setting/getting time on O3D cameras
* Added support for setting temporary application parameters. Please note, that
  if the device does not support this, it may "fail silently", so, a
  closed-loop check by the user is recommended.

## Changes between ifm3d 0.5.0 and 0.6.0

* Added the pcic client feature from `libo3d3xx`
* Added the ability to dump on-camera tracelogs including an interface to this
  capability via the `trace` subcommand to the `ifm3d` command-line tool.

## Changes between ifm3d 0.4.0 and 0.5.0

* Added `swupdate` subcommand in the tools module
* Added image module support to Windows build

## Changes between ifm3d 0.3.3 and 0.4.0

* Added modules/tools/contrib with bash completions for ifm3d

## Changes between ifm3d 0.3.2 and 0.3.3

* Windows build support (should have been a bump to 0.4.0)

## Changes between ifm3d 0.3.1 and 0.3.2

* CMake build scripts now look for opencv in tools module since the image
  buffer header includes an opencv header

## Changes between ifm3d 0.3.0 and 0.3.1

* Fixed regression on 14.04 - no compiler support for std::put_time (#3)

## Changes between ifm3d 0.2.0 and 0.3.0

* Support for NTP (on O3X)
* Added simple viewer sub-command to the `ifm3d` command-line program. This
  viewer will render the point cloud and color each pixel with the normalized
  amplitude value registered to that point.

## Changes between ifm3d 0.1.0 and 0.2.0

* Added software trigger support to O3X
* Added support for ifm Vision Assistant compatible import/export functions for
  O3X cameras
* Optimization to `ifm3d` cmd line tool when passed either `--help` or
  `version`. It will no longer try to connect to the device first, which makes
  this much more responsive and convenient for when no h/w is plugged in.
* Added the ability to explicitly choose OpenCV 2.4 or OpenCV 3 at
  cmake/compile time.
* Modifications to enable the library to build under Ubuntu 14.04 (C++11
  instead of C++14 and gcc 4.8. Big thanks to @aaronhoy at Fetch Robotics for
  [his work](https://github.com/aaronhoy/ifm3d/commit/b2e894e3a4f4afc227b7d33993f0a85e4078d513)
* Added a new build-time utility
  [ifm3d-dpkg-deps.py](cmake/utils/ifm3d-dpkg-deps.py.in) to auto-generate
  debian dependencies for the binary packages. This is needed because, for how
  we are building multiple shared libraries across multiple debian packages,
  cmake's stanard wrapper to `dpkg-shlibdeps` does not work for us (for several
  reasons).

## This file has started tracking ifm3d at 0.1.0

* Initial (alpha) release
