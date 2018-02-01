
ifm3d Examples
==================

This directory contains example programs that utilize `ifm3d`. The
intention is to create standalone programs that illustrate one very specific
concept in order to serve the purpose of letting developers ramp up quickly
with using the library. The build infrastructure in this directory is minimal
and the programs are intended to be run in place. Additonally, unless
specifically stated otherwise, things like performance and robust error
handling are not demonstrated. The purpose is to clearly illustrate the task
without clouding it with the details of real-world software engineering --
unless, of course, that was the point of the example.

It is expected that this library of examples will grow over time in response to
common themes we see on the issue tracker.

Building the examples
----------------------

Assuming you are starting from the top-level directory of this source
distribution:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

The example module is enabled by default. It can be excluded from the build by defining ``-DBUILD_MODULE_EXAMPLES=OFF``

    $ mkdir build
    $ cd build
    $ cmake -DBUILD_MODULE_EXAMPLES=OFF ..
    $ make


What is included?
-----------------

* [ex-file_io](ex-file_io.cpp) Shows how to capture data from the camera and
  write the images to disk. In this example, the amplitude and radial distance
  image are written out as PNG files and the point cloud is written as a PCD.
* [ex-getmac](ex-getmac.cpp)
  Request the MAC address from the camera. The MAC address can be used as
  a unique identifier.
* [ex-timestamp](ex-timestamp.cpp)
 Request some frames from the camera and write the timestamps to stdout
* [ex-exposure_times](ex-exposure_times.cpp) Shows how to change imager
  exposure times on the fly while streaming in pixel data and validating the
  setting of the exposure times registered to the frame data.