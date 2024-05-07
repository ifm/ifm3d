ifm3d Software Compatibility Matrix
-----------------------------------
| ifm3d version| O3D Firmware Version| O3X Firmware Version| O3R Firmware Version| Ubuntu Linux Version| Notes|
|:-------------|:--------------------|:--------------------|:--------------------|:------------|:-----|
| 0.1.0| 1.6.2114| 0.1.4| n/a| 16.04| Initial (beta) release|
| 0.2.0| 1.6.2114| 0.1.16, 0.1.20| n/a| 14.04, 16.04| Software triggering (O3X), support for Ubuntu 14.04|
| 0.3.0| 1.6.2114| 0.1.20| n/a| 14.04, 16.04| Parsing extrinsics (O3D), NTP support (O3X), Simple GUI|
| 0.3.1| 1.6.2114| 0.1.20| n/a| 14.04, 16.04| Fixed 14.04 regression (std::put_time)|
| 0.3.2| 1.6.2114| 0.1.20| n/a| 14.04, 16.04| Patch to tools module build script|
| 0.3.3| 1.6.2114| 0.1.20| n/a| 14.04, 16.04| Windows build support|
| 0.4.0| 1.6.2114| 0.1.20| n/a| 14.04, 16.04| Added bash completions for ifm3d command line tool|
| 0.5.0| 1.6.2114| 0.1.20| n/a| 14.04, 16.04| Added firmware flashing to ifm3d command line, Windows support for  image module|
| 0.6.0| 1.6.2114| 1.0.62| n/a| 14.04, 16.04| Added PCIC client, ability to dump camera trace logs|
| 0.7.0| 1.6.2114, 1.20.973| 1.0.62| n/a| 14.04, 16.04| Timestamping of image buffers, host/device time sync for O3D, changing  application parameters on-the-fly|
| 0.8.1| 1.6.2114, 1.20.973| 1.0.62| n/a| 14.04, 16.04| Register illumination temperature to frame data|
| 0.8.2| 1.6.2114, 1.20.973| 1.0.62| n/a| 14.04, 16.04| Patches to windows build|
| 0.9.0| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04| Dropped support for Ubuntu 14.04, moved to C++14, CMake 3.5, removed  viewer application, PCL is now optional, provides a framework for  implementing alternative image containers, ability to set password on  camera.|
| 0.9.1| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04| Removed some additional Boost dependencies|
| 0.9.2| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04, 18.04| Added support for Ubuntu 18.04|
| 0.9.3| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Docs and minor patches|
| 0.10.0| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Session IDs can be explicitly set and shared across camera  instances. Speed improvements to JSON imports.|
| 0.11.0| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Added a jitter subcommand to ifm3d, camera intrinsics exposed|
| 0.11.1| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Bad pixels are now flagged by the driver, OpenCV image container module  headers can be included in multiple translation units, and other minor bug  fixes.|
| 0.11.2| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Less verbose logging in `framegrabber` for better memory, disk, cpu  consumption on long-running embedded systems|
| 0.12.0| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522, 1.23.2848| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Inverse intrinsic parameters from O3D cameras|
| 0.13.0| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522, 1.23.2848,  1.25.4073| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Introduced `pybind11` module to provide ifm3d Python bindings|
| 0.14.0| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522, 1.23.2848,  1.25.4073, 1.30.4123| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Introduced `swupdater` module for firmware update utilities|
| 0.14.1| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522, 1.23.2848,  1.25.4073, 1.30.4123| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Timeouts fixes in `swupdater` module, JSON library update|
| 0.15.0| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522, 1.23.2848,  1.25.4073| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Interface added to grab json_model data of application output|
| 0.15.1| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522, 1.23.2848,  1.25.4073| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Minor updates to allow for cross-compiling ifm3d for the O3D3XX|
| 0.16.0 (Fine to use, but the `FrameGrabberUdp` module is  not supported by firmware and is removed in the next version of ifm3d)| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522, 1.23.2848,  1.25.4073, 1.30.4123| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Introduced `framegrabberudp` module for usage with UDP-enabled camera  firmwares|
| 0.17.0| 1.6.2114, 1.8.769, 1.20.1138, 1.23.1506, 1.23.1522, 1.23.2848,  1.25.4073, 1.30.4123| 1.0.111, 1.0.122, 1.0.126| n/a| 16.04,18.04| Bugfixes and removed `FrameGrabberUdp` module|
| 0.18.0| 1.6.2114, 1.23.1522, 1.23.1522, 1.23.2848, 1.30.4123, 1.30.5309, 1.60.7020| 1.0.122, 1.0.126, 1.0.156| n/a| 16.04,18.04,20.04| Expanded support matrix for platforms/archs/firmwares|
| 0.20.0| 1.6.2114, 1.23.1522, 1.23.1522, 1.23.2848, 1.30.4123, 1.30.5309| 1.0.122, 1.0.126, 1.0.156| n/a| 16.04,18.04,20.04| Removed Boost dependence,Added clang-format,SPDX license Headers|
| 0.92.0| Not tested| Not tested| 0.13.11, 0.14.23| 16.04,18.04,20.04| early adopter release for O3R|
| 0.93.0| Not tested| Not tested| 13.11, 0.14.23| 20.04| early adopter release for O3R|
| 1.0.0| Not tested| Not tested| 13.11, 0.14.23, 0.16.23| 20.04| Official release for O3R|
| 1.1.0| Not tested| Not tested| 13.11, 0.14.23, 0.16.23| 20.04, 22.04| Provides documentation update, bugfixes and .debs files|
| 1.1.1| Not tested| Not tested| 13.11, 0.14.23, 0.16.23| 20.04, 22.04| Bugfixes|
| 1.2.6| Not tested| Not tested| 1.0.14 | 20.04, 22.04| Changes to the Python bindings and the JSON implementation|
| 1.3.2| Not tested| Not tested| 1.0.14| 20.04, 22.04| Add customer logger module|
| 1.3.3| Not tested| Not tested| 1.0.14| 20.04, 22.04| Bugfixes|
| 1.4.0| Not tested| Not tested| 1.0.14| 20.04, 22.04| Experimental support for O3D and O3X|
| 1.4.1| Not tested| Not tested| 1.0.14| 20.04, 22.04| Minor changes|
| 1.4.2| Not tested| Not tested| 1.1.30| 20.04, 22.04| Minor changes|
| 1.4.3| Not tested| Not tested| 1.1.30| 20.04, 22.04| Minor changes|