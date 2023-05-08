
# ifm3d - Command Line Tool

## Overview

The `ifm3d` (c++) and `ifm3dpy` (python) ship with a command line tool of the same name. The `ifm3d` command
line tool is used to both introspect the state of a camera as well as mutate
parameters. To carry out a particular task, you evoke one of the `ifm3d` or `ifm3dpy`
*subcommands*. To get a listing of available subcommands, you can pass the
`--help` option.

> Note: the `ifm3d` and `ifm3dpy` CLIs are the same tool under a different name, related to the parent package. If you installed the c++ library `ifm3d`, the CLI is called `ifm3d`. If you installed the python library `ifm3dpy`, the CLI is called `ifm3dpy`. In the instructions below, you can change `ifm3d` for `ifm3dpy` when using the latter tool, with identical results.

```$ ifm3d --help
ifm3d: version=1.2.6


Usage:
  ifm3d [<global options>] <command> [<args>]

 global options:
  -h, --help             Produce this help message and exit
      --ip arg           IP address of the sensor (default: 192.168.0.69)
      --xmlrpc-port arg  XMLRPC port of the sensor (default: 80)
      --password arg     Password for establishing an edit-session with the
                         sensor (default: )

These are common commands used in various situations:

    app-types     List the application types supported by the sensor.

    config        Configure sensor settings from a JSON description of
                  the desired sensor state. See also `dump'.

    cp            Create a new application on the sensor,
                  bootstrapped from a copy of an existing one.

    diagnostic    Access the device diagnostic information

    discover      Discover ifm devices on the network.

    dump          Serialize the sensor state to JSON.

    export        Export an application or whole sensor configuration
                  into a format compatible with ifm Vision Assistant.
      
    hz            Compute the actual frequency at which the FrameGrabber
                  is running.
      
    imager-types  List the imager types supported by the sensor.

    import        Import an application or whole sensor configuration
                  that is compatible with ifm Vision Assistant's export
                  format.
      
    jitter        Collects statistics on framegrabber (and optionally, image
                  construction) jitter.
      
    jsonschema    Gets current JSON schema configuration.
    
    ls            Lists the applications currently installed on
                  the sensor.

    passwd        Sets the password on the sensor.

    reboot        Reboot the sensor, potentially into recovery
                  mode. Recovery mode is useful for putting the
                  sensor into a state where it can be flashed
                  with new firmware.

    reset         Reset the sensor to factory defaults.

    rm            Deletes an application from the sensor.
      
    schema        Construct and analyze image acquisition schema masks.
      
    swupdate      Perform a firmware update on the camera. Please ensure
                  that the camera is booted to recovery beforehand.

    time          Get/set the current time on the camera.

    trace         Get trace messages from the internal camera trace buffer.

For bug reports, please see:
https://github.com/ifm/ifm3d/issues
```

As it is reported in the help output above, the `ifm3d` command line program
accepts 1) a set of *global arguments* which control the particular platform you
wish to communicate with; 2) a subcommand; and 3) arguments to the
subcommand. To get a listing of the particular arguments accepted by a
subcommand, you can pass the `--help` option to the subcommand. For exemplary
purposes, let's list the options accepted by the `dump` subcommand.

```
$ ifm3d dump --help

Usage:
  ifm3d [<global options>] dump [<dump options>]

 global options:
  -h, --help             Produce this help message and exit
      --ip arg           IP address of the sensor (default: 192.168.0.69)
      --xmlrpc-port arg  XMLRPC port of the sensor (default: 80)
      --password arg     Password for establishing an edit-session with the
                         sensor (default: )

```

As is shown above, `dump` takes the IP address of the device, the xmlrpc and the password (if these are non-default).

We now walk through a couple of simple examples of using `ifm3d`. This is not an
exhaustive tutorial on `ifm3d` but rather intended to give a sense of how to
use the tool. The concepts apply broadly to all of the subcommands.
