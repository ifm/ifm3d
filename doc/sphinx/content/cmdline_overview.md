
# ifm3d - Command Line Tool

## Overview

`ifm3d` ships with a command line tool of the same name. The `ifm3d` command
line tool is used to both introspect the state of a camera as well as mutate
parameters. To carry out a particular task, you evoke one of the `ifm3d`
*subcommands*. To get a listing of available subcommands, you can pass the
`--help` option.

```
$ ifm3d --help
ifm3d: version=0.20.0


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

    ls            Lists the applications currently installed on
                  the sensor.

    passwd        Sets the password on the sensor.

    reboot        Reboot the sensor, potentially into recovery
                  mode (no recovery mode for O3R).
                  Recovery mode is useful for putting the
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
purposes, let's list the options accepted by the `cp` subcommand.

```
$ ifm3d cp --help
Usage:
  ifm3d [<global options>] cp [<cp options>]

 global options:
  -h, --help             Produce this help message and exit
      --ip arg           IP address of the sensor (default: 192.168.0.69)
      --xmlrpc-port arg  XMLRPC port of the sensor (default: 80)
      --password arg     Password for establishing an edit-session with the
                         sensor (default: )

 cp options:
      --index arg  Index of source application to copy (default: -1)
```

As is shown above, `cp` takes a source application index to copy from. Note that the concept of applications is deprecated for the O3R platform.

We now walk through a couple of simple examples of using `ifm3d`. This is not an
exhaustive tutorial on `ifm3d` but rather intended to give a sense of how to
use the tool. The concepts apply broadly to all of the subcommands.
