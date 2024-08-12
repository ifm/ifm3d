# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Add `ovp8xx`,`o3x1xx` and `o3d3xx` and `discover` commands as a primary level subcommands to ifm3d command line interface

### Changed
- Split CLI into device specific `ovp8xx`,`o3x1xx`,`o3x2xx`,`o3d3xx` subcommands

### Changed
- Use CLI11 library instead of cxxopts for ifm3d command line interface

### Added
- The documentation about the different available `buffer_id` has been updated to provide the complete list for each compatible device.
- add `O3R_ODS_RENDERED_ZONES` `buffer_id`

## 1.6.0 - 2024-06-10
### Changed
- Unification of shared libraries into `libifm3d`, see the migration guide for details

### Fixed
- Updated `TOFInfoV3` structure for exposure time, mode and imager

## 1.5.3 - 2024-04-05
### Fixed
- Fix receiving frames with default format before PCIC p command
    has been acknowledged

## 1.5.2 - 2024-02-20
### Added
- add `O3R_RESULT_JSON`, `O3R_RESULT_ARRAY2D`, `O3R_RESULT_IMU` `buffer_ids`

### Fixed
- Fix curl error from `SWUpdate`

## 1.5.1 - 2024-02-07
### Fixed
- release pipeline

## 1.5.0 - 2024-02-07 [YANKED] error during release - use 1.5.1
### Added
- Python 3.12 builds
- Parsing of V3 chunks
- Ability to access V3 chunk metadata (see Frame::Metadata)
- Ability to access multiple chunks with the same id in a frame (see Frame::GetBuffer and Frame::GetBufferCount)
- O3R_RESULT_JSON and O3R_RESULT_ARRAY2D chunk ids

### Changes
- Use .deb files in Dockerfile instead of building from sources
- Increase timeout for O3R set calls up to 15s

### Fixed
- Fix installation prefix [issue](https://github.com/ifm/ifm3d/issues/434) from tools
- Add WebSockets++ third-party dependency using FetchContent_Declare
- Fix device discovery [issue](https://github.com/ifm/ifm3d/discussions/441)

## 1.4.3 - 2023-09-28
### Fixed
- Docker deployment 
- Add missing deserialize module to docker containers

## 1.4.2 - 2023-09-15

### Fixed
- Fix data grabbing from diagnostic port
- `SWUpdate` fix installing O3R updates from stdin
- Fix compatibility for O3R devices with Firmware version below 1.1.x

## 1.4.1 - 2023-08-31
### Fixed
- O3R.resolve_config unable to resolve leaf values
- O3R::Ports and O3R::Port now wrap internal errors in ifm3d::Error

### Added 
- O3R add support for `SaveInit` with JSON pointers as arguments
- `ifm3d::Error` codes are now exposed to Python 
- Python bindings for ifm3d logging

## 1.4.0 - 2023-08-02

**NOTE** : O3D and O3X support is currently experimental- Use at your own risk!.

### Added
- Add support for O3D3xx and O3X devices
- Extend Ports() to return app port
- Extend Port() to return information of given app port

### Changed
- Updated cxxopts library to version 3.1.1 and used through `FetchContent`


 
## 1.3.3 - 2023-06-22

### Fixed
- Fixed the error reporting through `onError` callback

## 1.3.2 - 2023-06-16
### Added
- Add environment variable `IFM3D_SWUPDATE_CURL_TIMEOUT` for curl transaction timeout during software update
- Add Project description and links to PyPI page
- Add an example on how to use the deserializer module

### Fixed
- Release versions wrongly marked as +dirty

### Changed
- Replaced glog with ifm3d::Logger, see the migration guide for details
- The JSON module has been moved from `device` to `common`. Update your includes accordingly: use `#include <ifm3d/common/json.hpp` instead of `#include <ifm3d/device/json.hpp`.

## 1.3.1 - 2023-06-16 [YANKED] error during release - use 1.3.2

## 1.3.0 - 2023-06-16 [YANKED] error during release - use 1.3.2

## 1.2.6 - 2023-04-05
### Fixes
- Fix state of the FrameGrabber not being reset correctly after Stop() causing receive failures on subsequent Start()s

### Added
- Generate and distribute stubs with Python wheel package
- Add MAC address in ifm3d discover command output

## 1.2.5 - 2023-04-03
### Fixes 
- Fixed a crash in `FrameGrabber` when the p command is called multiple times
- Fixed potential crash when waiting for the future returned by `FrameGrabber::WaitForFrame()`

## 1.2.4 - 2023-03-28
### Changes
- Increase network timeout for the `O3R::Set()` call to 10 seconds

### Fixes 
- Fixed potential crash during device discovery
- Fixed possible Segmentation fault when calling `FrameGrabber::Stop()` directly after `FrameGrabber::Start()`
- Fixed typo in `RGBInfoV1`, `TOFInfoV4` and `TOFInfoV3`: `extrisic_optic_to_user` -> `extrinsic_optic_to_user`

## 1.2.3 - 2023-03-17
### Fixes 
- Fixed segmentation fault when assigning a `AsyncNotification` or `AsyncError` handler while the `FrameGrabber` is not running
- Added missing alias for `ifm3dpy.Error` to `ifm3dpy.device.Error`
- Fix device discovery requiring manual firewall rule on windows

### Added
- CI job for release of ifm3d

## 1.2.2 - 2023-03-06
### Added
- Added `Frame::FrameCount` to access the frame counter value
- Added API to enable disable masking in `FrameGrabber`

### Changed 
- Split the Python bindings into submodules, see the migration guide for details. *Warning:* this requires an update of existing codebases.
- Convert `SWUpdater` Python bindings naming to `snake_case`, see the migration guide for details. *Warning:* this requires an update of existing codebases.
- `FrameGrabber::Start` now returns a future resolving once the `FrameGrabber` is ready to receive `Frames`

### Fixes 
- Fix a bug that could cause the update process to fail with a hash mismatch error on certain network configurations
- Fix a bug preventing the `FrameGrabber` from being `Start`ed after it was previously `Stop`ped
- Fix a typo in the `transform_cell_center_to_user` variable (previously called `transfor_cell_center_to_user`).
- Add missing Python binding for `O3R::ResolveConfig`
- Fix CMake targets for deserialize module

## 1.2.1 - 2023-02-09
### Fixes 
- Fix a bug that could result in an endless loop when receiving PCIC tickets
- Fix a crash when receiving PCIC data without any chunks

## 1.2.0 - 2023-02-03

### Changed
- Upgraded `nlohmann::json` to version 3.11.2
- Removed JSON from the global namespace and moved `nlohmann::json` to the `ifm3d::json` namespace

### Fixes 
- Missing Python bindings for `O3R::Port` and `O3R::Ports`
- `CONFIDENCE_IMAGE` image is not requested automatically anymore unless it's required for generating the requested chunks

### Added
- `O3R::RebootToRecovery` to reboot supported O3R devices into recovery mode
- Support for O3R recovery based updates
- Add `size()` interface in `ifm3d::Buffer`
- Add Deserialize module
  - Add struct `TOFInfoV3`
  - Add struct `TOFInfoV4`
  - Add struct `RGBInfoV1`
  - Add struct `ODSInfoV1`
  - Add struct `ODSOccupancyGridV1`

## 1.1.1 - 2022-12-09
### Fixes 
- Data grabbing [issue](https://github.com/ifm/ifm3d/issues/377) with ifm3dpy v1.1.0

## 1.1.0 - 2022-12-02
### Added
- Change `FrameGrabber::Stop` to non blocking call and now returns `std::future<void>`
- Add `onError` callback for error reporting in streaming mode of `Framegrabber`
- Support for retrieving O3R diagnostics over XMLRPC
- Support for Python 3.11
- Support for Ubuntu 22.04
- Python binding: `FrameGrabber.sw_trigger()`

### Fixes
- `XYZ_IMAGE` coordinates were actually ZXY instead of XYZ, this has been corrected.

## 1.0.1 - 2022-10-14
### Added
- Auto closure of stale issue on GitHub
- Support for ODS schema
- Asynchronous notification support

### Fixes
- ifm3d and ifm3dpy documentation update
- Confidence buffer available for user
- Reflectivity buffer support in schema
- Distance noise buffer in float format
- Exception if buffer is not available in frame
- Extrinsic values
- Linux .deb file now depends on local installed libraries

## 1.0.0 - 2022-09-08
### Added
- Example update showing usage of `FrameGrabber` API
- Visibility attribute added for ifm3d API
- Playground example for CMake users
- Python API renamed as per C++ changes
- `CameraO3D`, `CameraO3R`, `CameraO3X` renamed as `O3D`, `O3R`, `O3X` respectively
- `Camera` is renamed as `LegacyDevice`
- `CameraBase` is renamed as `Device`
- `image_id` is renamed as `buffer_id`
- `Image` is renamed as `Buffer`
- `camera` module name changed to `device`
- Example list update
- Windows installer support for ifm3d
- ifm3dpy documentation update
- Software trigger support for the `framegrabber` module
- Software trigger error reporting on execution failure
- Asynchronous error support for `framegrabber` module
- Schema support for `framegrabber` module
- OSS compliance added
- Windows build instruction update
- Major architecture changes in the `framegrabber` module
- `Swupdater` support for ifm3dpy
- File(.swu) streaming for updating firmware through `swupdater`
- Multiple timestamp support in ifm3dpy
- O3R support for intrinsic calibration model type 2 (fisheye distortion model)

### Fixed
- Error code from 7 digit to 6 digit for `Lib` error codes
- Discover app for O3R devices
- `ifm3d::tools` compatibility for O3R devices


## 0.93.0 - 2022-02-17
### Added
- Document for schema
- Document for O3X parameters
- Added functionality to get timestamp at which data is send over Ethernet
- Support for distance noise image for O3X Devices
- Support for latest O3X firmware (1.1.190)
- New Parameters for O3X device: `AbsDistStraylightThreshold`, `EnableStraylightCorrection`, `EnableNoiseEstimation`,
  `CompensateAmbientLightDrift`, `DistNoiseThreshold`, `EnableNoiseEstimation`, `RelAmpStraylightThreshold`
- Added the O3X FW 1.1.166 to the compatibility list
- Added compatibility list to software compatibility document
- `error_t::message()` function to retrieve details about exceptions
- Custom Python exception type: `ifm3dpy.Error`
- Added timeout option in `swupdate` command of tools

### Fixed
- O3R FW detection

### Changed
- The conversion of the camera frame is now a compile time option (Use `-DUSE_LEGACY_COORDINATES=ON` to keep the old behaviour)
- `O3RCamera::Port` & `O3RCamera::Ports` methods to get information (PCIC port & type) of connected ports
- `O3RCamera::ResolveConfig` convenience method to access specific parts of the configuration

## 0.92.0 - 2021-10-22
### Added
- HTTP 407 Proxy authentication required error detection
- Python 3.10 builds
- Docker image on DockerHub: ifmrobotics/ifm3d
- Docker image on GHCR: ghcr.io/ifm/ifm3d:latest
- Basic usage tutorials

### Fixed
- O3R broken `XYZImage` 
- O3R `getInit()` method
- Use the correct base image for arm64 based containers

### Changed
- Docker images now build ifm3d in Release mode 

## 0.91.0 - 2021-10-05

### Added
- O3R specific methods

## 0.90.2 - 2021-09-16

- `stlImage` module (`Image` container based on STL)
- Removed copying of the tools header
- Example to upload docker container to O3R

## 0.90.1 - 2021-08-17

### Added
- Basic C++ tutorials
- Support for the new JSON based XML-RPC interface
- Support for 2D image data
- `ifm3dpy_viewer` Python example
- Generate version based on last tag and commits since

### Changed
- Split `Camera` implementation into multiple classes
- `IsO3D`/`IsO3X`/`IsO3R` replaced by `WhoAmI`/`AmI` functions

## 0.90.0 - 2021-01-18

### Added

- Basic O3R support
- Support for the compressed image format introduced for O3R
- Initial IPv4 Discovery in the ifm3d command line tool

### Removed

- Hardcoded compiler flags for Linux
- Copy of the header files during CMake build

## Changes between ifm3d 0.18.0 and 0.20.0 [Unreleased]
* Added clang format support for formatting
* Changed license headers to SPDX format
* Embedded third-party libraries Asio and cxxopts
  * Removed boost from dependency list
* Support user defined port for `camera`, `fg`, `swupdater` module
  * This enable ifm3d to connect to devices behind NAT router
* Added example for NTP to command line usage
* Added build jobs in GitHub actions
  * Windows VS 2019
  * Ubuntu 20.04 
* Bugfixes
  * #284 ifm3d compiling error at `swupdater` app with VS2019 and Windows
  * #283 Imported target `ifm3d::image` includes non-existent path `/usr/include/opencv`

## Changes between ifm3d 0.17.0 and 0.18.0
* Support for latest O3D3XX firmware (1.30.5309)
* Support for latest O3X firmware (1.0.156)
* Deprecated ROS-specific apt repositories
* Deprecated Python2 support
* Support for Ubuntu 20.04 Focal Fossa
* Packaged and released ifm3d as a Snap
* Added support for Ubuntu ARM64
* Added support for L4T (Linux for Tegra) JetPack 4.3 and 4.4
* Improved Windows build instructions
* Created GitHub Actions CI workflows
* Bugfixes:
  * #190 - Added missing include for Windows build

## Changes between ifm3d 0.16.0 and 0.17.0
* Reverted changes in 0.16.0 (`FrameGrabberUdp` -- No viable path to UDP
  implementation in FW)
* Bugfixes
  * Issue with libcurl usage on 32bit targets
  * Corrected minimum firmware version required for inverse intrinsics
  * Corrected handling of spurious wakes in FrameGrabber
  * Fixed `ComputeCartesian` Python unit test to properly blank out invalid
    pixels
  * Changed `build` Dockerfiles to use pip for NumPy/pytest
  * Fixed race condition in `PCICClient` unit tests
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
* New module: `swupdater` -- utilities for updating camera firmware
    * Ported functionality from `swupdate` command into its own library for
      programmatic consumption.
    * Updated certain semantics of the `swupdate` command in the `tools`
      module to match those of the other ifm3d `tools` commands
        * Updated command line switch naming to match other ifm3d tools:
          * `check` subcommand now invoked by `-c` or `--check`
          * `reboot` subcommand now invoked by `-r` or `--reboot`
        * `file` subcommand will now test for recovery and automatically
          reboot the device into recovery as needed.
* Disabled FrameGrabber's `InverseIntrinsicParamSchema` test due to suspected
  false failures. Test case will be investigated and re-opened in a future
  release.
* Fixed issues with unit test scripts on Windows
* Fixed Windows build documentation
  * Added `BUILD_SHARED_LIBS` definition to `glog` to address issues with
    logging to STDERR in Windows binaries
  * Parameterized the CMake generator for easier building when multiple
    versions of MSVC are installed concurrently

## Changes between ifm3d 0.12.0 and 0.13.0
* Honor semantics of CMake's `BUILD_SHARED_LIBS` flag (ON by default). Setting
  to off will build and link against ifm3d modules as static libraries.
* New module: `pybind11` -- Python bindings for the C++ API

## Changes between ifm3d 0.11.2 and 0.12.0

* Fixes to build infrastructure in support of windows unit tests
* Added support to retrieve the inverse intrinsic parameters from O3D3xx
  cameras

## Changes between ifm3d 0.11.1 and 0.11.2

* Bugfix for #111, moved a log message in `framegrabber` to `IFM3D_PROTO_DEBUG` to
  keep noise level low when running an O3X for extended periods of time.
* Changed flagging bad pixels to always be `0` regardless of data type. Users
  could always consult the confidence image themselves and discriminate between
  a true `0` (not possible) and a bad pixel which they could then transform to
  `nan` or whatever other sentinel makes sense for their application.

## Changes between ifm3d 0.11.0 and 0.11.1

* Bugfix for #103 ``header is not in the correct format`` when ``make check`` is
  executed against FW 1.6.2114
* Bugfix for #107 Allows OpenCV module headers to be included in more than one
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
* Added troubleshoot guide
* Added OpenCV module build instruction for windows
* Added minimum MSVC version requirement
* Added prerequisite packages list for building ifm3d
* Changed warning message in `framegrabber` from `WARNING` to `IFM3D_TRACE`
  severity level
* Updated `swupdate` command with --check (recovery mode check) and -r (reboot to
  productive mode)

## Changes between ifm3d 0.9.1 and 0.9.2

* Added support for the Ubuntu 18.04
* Added glog support in the CMake configuration files

## Changes between ifm3d 0.9.0 and 0.9.1

* Removed some additional Boost dependencies

## Changes between ifm3d 0.8.3 and 0.9.0

* Version number handling is now done in the CMake `project` command in the
  top-level CMakeLists.txt file
* Dropped support for Ubuntu 14.04
* Increased CMake requirements to 3.5
* Increased compiler requirements to C++14
* Moved `examples` module into new project
  [ifm3d-examples](https://github.com/lovepark/ifm3d-examples)
* Moved `viewer` sub-command out of the ifm3d project. This is to decrease the
  PCL dependencies (see Issue #42). A new project will be created in support of
  this viewer application:
  [ifm3d-pcl-viewer](https://github.com/lovepark/ifm3d-pcl-viewer)
* Updated JSON parsing library to 3.1.2
* By default, `pcicclient` module is now `OFF`.
* Pixel-parsing framework has been significantly refactored. Sub-system
  specific docs for image container implementers have been provided in the
  `doc` folder.
* Updated the `ImageBuffer` to conform to the new pixel-publishing
  architecture.
* Initial implementation of an OpenCV-only (that is, no PCL) image container. This
  is the `opencv` module of the `ifm3d` project.
* Added a `passwd` subcommand to `ifm3d`

## Changes between ifm3d 0.8.2 and 0.8.3

* Fixed a CMake regression regarding `-std=c++11` flags passed to the compiler;
  surfaces on old versions of CMake, that is, in Ubuntu 14.04

## Changes between ifm3d 0.8.1 and 0.8.2

* Patch to Windows build
* Better semantic versioning parsing of camera firmware

## Changes between ifm3d 0.8.0 and 0.8.1

* Reverted Windows build changes due to how it broke packaging on Linux

## Changes between ifm3d 0.7.0 and 0.8.0

* Illumination temperature is registered to frame data

## Changes between ifm3d 0.6.0 and 0.7.0

* Added timestamping of image buffers
* Added support for setting/getting time on O3D cameras
* Added support for setting temporary application parameters. Please note, that
  if the device does not support this, it may "fail silently," so, a
  closed-loop check by the user is recommended.

## Changes between ifm3d 0.5.0 and 0.6.0

* Added the PCIC client feature from `libo3d3xx`
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

* CMake build scripts now look for OpenCV in tools module since the image
  buffer header includes an OpenCV header

## Changes between ifm3d 0.3.0 and 0.3.1

* Fixed regression on 14.04 - no compiler support for `std::put_time` (#3)

## Changes between ifm3d 0.2.0 and 0.3.0

* Support for NTP (on O3X)
* Added simple viewer sub-command to the `ifm3d` command-line program. This
  viewer will render the point cloud and color each pixel with the normalized
  amplitude value registered to that point.

## Changes between ifm3d 0.1.0 and 0.2.0

* Added software trigger support to O3X
* Added support for ifm Vision Assistant compatible import/export functions for
  O3X cameras
* Optimization to `ifm3d` command line tool when passed either `--help` or
  `version`. It will no longer try to connect to the device first, which makes
  this much more responsive and convenient for when no h/w is plugged in.
* Added the ability to explicitly choose OpenCV 2.4 or OpenCV 3 at
  CMake/compile time.
* Modifications to enable the library to build under Ubuntu 14.04 (C++11
  instead of C++14 and GCC 4.8. Big thanks to @aaronhoy at Fetch Robotics for
  [his work](https://github.com/aaronhoy/ifm3d/commit/b2e894e3a4f4afc227b7d33993f0a85e4078d513)
* Added a new build-time utility
  [ifm3d-dpkg-deps.py](cmake/utils/ifm3d-dpkg-deps.py.in) to auto-generate
  Debian dependencies for the binary packages. This is needed because, for how
  we are building multiple shared libraries across multiple Debian packages,
  CMake's standard wrapper to `dpkg-shlibdeps` does not work for us (for several
  reasons).

## This file has started tracking ifm3d at 0.1.0

* Initial (alpha) release
