
ifm3d
=====
Library and utilities for working with ifm pmd-based 3D ToF Cameras.


Software Compatibility Matrix
-----------------------------
<table>
  <tr>
    <th>ifm3d version</th>
    <th>O3D Firmware Version</th>
    <th>O3X Firmware Version</th>
    <th>Notes</th>
  </tr>
  <tr>
    <td>0.1.0</td>
    <td>1.6.2114</td>
    <td>0.1.4</td>
    <td>Initial (beta) release</td>
  </tr>
  <tr>
    <td>0.2.0</td>
    <td>1.6.2114</td>
    <td>0.1.16</td>
    <td></td>
  </tr>
</table>

Disclaimer
----------
This software is under active development and not yet ready for production
systems. If you are looking for a C++ sensor interface to the ifm O3D303,
please consider using [libo3d3xx](https://github.com/lovepark/libo3d3xx).

If you are an early-adopter user of the new O3X, please keep reading.


Organization of the Software
----------------------------
The ifm3d software is organized into modules, they are:

<table>
  <tr>
    <th>Module Name</th>
    <th>Description</th>
  </tr>
  <tr>
    <td>Camera</td>
    <td>Provides an implementation of the XMLRPC protocol for configuring the
  camera and PMD imager settings</td>
  </tr>
  <tr>
    <td>Framegrabber</td>
    <td>Provides an implementation of the PCIC protocol for streaming pixel
  data and triggered image acquisition.</td>
  </tr>
  <tr>
    <td>Image</td>
    <td>Provides a bridge from raw camera bytes to OpenCV and PCL image
  encodings.</td>
  </tr>
  <tr>
    <td>Tools</td>
    <td>Provides the ifm3d command line tool for manipulating and introspecting
  the hardware interactively. It is also suitable for usage within shell scripts.</td>
  </tr>
</table>


Installing the Software
-----------------------

### Building from source

**NOTE:** If you are on Ubuntu 16.04 LTS and you are running ROS, we highly
  recommend you just install
  [our supplied binaries](https://github.com/lovepark/ifm3d/releases/tag/v0.1.0). Further
  instructions are listed below.

If you plan to build the software from source, you will need to have the
following pre-requisites installed on your machine:

* [Boost](http://www.boost.org)
* [Gtest](https://github.com/google/googletest)
* [Glog](https://github.com/google/glog)
* [libxmlrpc](http://xmlrpc-c.sourceforge.net/)
* [CMake](http://www.cmake.org)
* [OpenCV](http://opencv.org)
* [PCL](http://pointclouds.org)

Building the software follows the usual cmake idiom of:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_INSTALL_PREFIX=/usr ..
$ make
$ make check
$ sudo make install
```

A few important notes when building from source:

* For the `make check` step, we highly recommend you run your unit tests
  against an O3D and **not** an O3X. Some of the tests will likely
  fail. However, this will not be an indication that there is a problem with
  the software. We are still characterizing this new sensor and we are working
  very closely with ifm to gain clarity on its expected behavior. Running the
  unit tests against an O3D however, should pass. If the O3D tests pass, you
  should be OK to install the software and run it against either an O3D or
  O3X.

* An alternative to the `make install` step (which we don't like very much
  ourselves) is to `make package`. This will create a set of deb files that you
  can then install with `dpkg`. We note however that our OpenCV dependency in
  the Debian control file is expressed as the OpenCV supplied with ROS, which
  you may not have installed. In which case, your easiest path forward may be
  to use the `make install` step as called out above.


### Installing binaries

If you are running on Ubuntu (16.04 LTS) and running ROS (Kinetic), binaries
have been supplied with the initial 0.1.0 release. They can be downloaded from
[here](https://github.com/lovepark/ifm3d/releases/tag/v0.1.0)

Before installing the supplied debs, you should first update your packages on
Ubuntu 16.04 LTS:

    $ sudo apt-get update
    $ sudo apt-get -u upgrade

You should then
[install ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu). We recommend
the `ros-kinetic-desktop-full` installation.

**NOTE:** ROS is not strictly necessary for using ifm3d. We recommend
  installing ROS when using our supplied binaries because (currently) our
  dependency on OpenCV in the Debian control file is expressed as the version
  of OpenCV 3 supplied with ROS Kinetic, namely
  `ros-kinetic-opencv3`. Experienced users can work around this restriction.

Once you have downloaded the supplied binaries, they can be installed as
follows (please note the order of installation):

    $ sudo dpkg -i ifm3d_0.1.0_amd64-camera.deb
    $ sudo dpkg -i ifm3d_0.1.0_amd64-framegrabber.deb
    $ sudo dpkg -i ifm3d_0.1.0_amd64-image.deb
    $ sudo dpkg -i ifm3d_0.1.0_amd64-tools.deb

If you run into a dependency issues, e.g., on a fresh Ubuntu install you may
see something like:

```
$ sudo dpkg -i ifm3d_0.1.0_amd64-camera.deb
Selecting previously unselected package ifm3d-camera.
(Reading database ... 249133 files and directories currently installed.)
Preparing to unpack ifm3d_0.1.0_amd64-camera.deb ...
Unpacking ifm3d-camera (0.1.0) ...
dpkg: dependency problems prevent configuration of ifm3d-camera:
 ifm3d-camera depends on libgoogle-glog0v5; however:
  Package libgoogle-glog0v5 is not installed.
 ifm3d-camera depends on libxmlrpc-c++8v5; however:
  Package libxmlrpc-c++8v5 is not installed.
```

You can do the following to resolve this:

    $ sudo apt-get -f install

At this point, if your camera is plugged in and powered on, you should be able
to (for example) dump its configuration. For example with an O3X plugged in:

```
$ ifm3d dump
{
  "ifm3d": {
    "Apps": [
      {
        "Description": "",
        "Id": "1299148885",
        "Imager": {
          "ExposureTime": "1000",
          "FrameRate": "5",
          "MaxAllowedFrameRate": "12.5",
          "MinimumAmplitude": "42",
          "SpatialFilter": {},
          "SpatialFilterType": "0",
          "SymmetryThreshold": "0.4",
          "TemporalFilter": {},
          "TemporalFilterType": "0",
          "Type": "1FRQ_1EXP_0GRAY"
        },
        "Index": "1",
        "Name": "",
        "OutputAmplitudeImage": "true",
        "OutputConfidenceImage": "true",
        "OutputDistanceImage": "true",
        "OutputGrayscaleImage": "false",
        "OutputXYZImage": "true",
        "TriggerMode": "1",
        "Type": "Camera"
      }
    ],
    "Device": {
      "ArticleNumber": "",
      "ArticleStatus": "??",
      "Description": "",
      "DeviceType": "1:512",
      "IPAddressConfig": "0",
      "ImageTimestampReference": "1489579229",
      "Name": "New sensor",
      "OperatingMode": "0",
      "PasswordActivated": "false",
      "SessionTimeout": "30",
      "UpTime": "1.59972222222222"
    },
    "Net": {
      "MACAddress": "00:02:01:40:54:09",
      "NetworkSpeed": "0",
      "StaticIPv4Address": "192.168.0.69",
      "StaticIPv4Gateway": "192.168.0.201",
      "StaticIPv4SubNetMask": "255.255.255.0",
      "UseDHCP": "false"
    },
    "_": {
      "Date": "Tue Mar 28 21:16:07 2017",
      "HWInfo": {
        "MACAddress": "00:02:01:40:54:09",
        "Mainboard": "#!03_M100_B01_12345678_008025483",
        "MiraSerial": "Not implemented"
      },
      "SWVersion": {
        "Algorithm_Version": "0.1.3",
        "Calibration_Device": "00:02:01:40:54:09",
        "Calibration_Version": "0.0.1",
        "ELDK": "GOLDENEYE_YOCTO_HARDFP-273-06d9c894636352a6c93711c7284d02b0c794a527",
        "IFM_Software": "0.1.4",
        "Linux": "Linux version 3.14.34-rt31-yocto-standard-00016-g5121435-dirty (jenkins@dettlx152) (gcc version 4.9.2 (GCC) ) #1 SMP PREEMPT RT Tue Mar 14 08:40:14 CET 2017",
        "Main_Application": "0.4.986"
      },
      "ifm3d_version": 100
    }
  }
}
```

Redirecting the above serialized json to a file would allow you to edit camera
parameters. Let's say you saved that data to a file called `o3x.json` and made
some modifications. To commit those changes to the hardware, you would:

    $ ifm3d config < o3x.json

You could also just edit a single parameter quite easily from the command
line. Referring to our dump above where the camera is set to stream data at 5
Hz, if we wanted to step the frame rate up to 10 Hz, you could:

    $ echo '{"Apps":[{"Index":"1","Imager":{"FrameRate":"10"}}]}' | ifm3d config

We can see that our setting took affect by dumping the camera configuration
again. This time, we *grep* our output using the handy json filtering tool
[jq](https://stedolan.github.io/jq):

```
$ ifm3d dump | jq .ifm3d.Apps[0].Imager.FrameRate
"10"
```

You can see what else is available via the `ifm3d` tool by running:

```
$ ifm3d --help
ifm3d: version=0.1.0
usage: ifm3d [<global options>] <command> [<args>]

global options:
  -h [ --help ]            Produce this help message and exit
  --ip arg (=192.168.0.69) IP address of the sensor
  --xmlrpc-port arg (=80)  XMLRPC port of the sensor
  --password arg           Password for establishing an edit-session with the
                           sensor


These are common commands used in various situations:

    cp          Create a new application on the sensor,
                bootstrapped from a copy of an existing one.

    config      Configure sensor settings from a JSON description of
                the desired sensor state. See also `dump'.

    dump        Serialize the sensor state to JSON.

    export      Export an application or whole sensor configuration
                into a format compatible with ifm Vision Assistant.

    hz          Compute the actual frequency at which the FrameGrabber
                is running.

    import      Import an application or whole sensor configuration
                that is compatible with ifm Vision Assistant's export
                format.

    ls          Lists the applications currently installed on
                the sensor.

    reboot      Reboot the sensor, potentially into recovery
                mode. Recovery mode is useful for putting the
                sensor into a state where it can be flashed
                with new firmware.

    reset       Reset the sensor to factory defaults.

    rm          Deletes an application from the sensor.

    schema      Construct and analyze image acquisition schema masks.


For bug reports, please see:
https://github.com/lovepark/ifm3d/issues
```

We also note that every sub-command also accepts the `--help` argument. For
example:

```
$ ifm3d reboot --help
usage: ifm3d [<global options>] reboot [<reboot options>]

global options:
  -h [ --help ]            Produce this help message and exit
  --ip arg (=192.168.0.69) IP address of the sensor
  --xmlrpc-port arg (=80)  XMLRPC port of the sensor
  --password arg           Password for establishing an edit-session with the
                           sensor

reboot options:
  -r [ --recovery ]     Reboot into recovery mode
```

For viewing the point cloud data, currently, our only supported visualization
tool is [rviz](http://wiki.ros.org/rviz). To utilize it, you will need to
install our [ifm3d ROS bindings](https://github.com/lovepark/ifm3d-ros) and
follow the instructions from there.


Known Issues, Bugs, and our TODO list
-------------------------------------

We greatly appreciate your choice to use our sensor interface software. As of
this writing, this software is very new. However, it is informed by many years
of developing and supporting
[libo3d3xx](https://github.com/lovepark/libo3d3xx), the de-facto C++ sensor
interface to the ifm O3D. We are committed to evolving this software package
into the production quality code that has made libo3d3xx the basis of many
commercial products with the additional benefit of providing a hardware
abstraction layer over top of *all* ifm pmd-based 3D ToF cameras.

We appreciate your feedback as you use the code. We will be tracking things
(including our TODO list) on the
[Github Issue Tracker](https://github.com/lovepark/ifm3d/issues).

LICENSE
-------

Please see the file called [LICENSE](LICENSE).

AUTHORS
-------

Tom Panzarella <tom@loveparkrobotics.com>
