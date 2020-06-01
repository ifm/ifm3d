
ifm3d - Command Line Tool
=========================

`ifm3d` ships with a command line tool of the same name. The `ifm3d` command
line tool is used to both introspect the state of a camera as well as mutate
parameters. To carry out a particular task, you envoke one of the `ifm3d`
*subcommands*. To get a listing of available subcommands, you can pass the
`--help` option.

```
$ ifm3d --help
ifm3d: version=0.18.0
usage: ifm3d [<global options>] <command> [<args>]

global options:
  -h [ --help ]            Produce this help message and exit
  --ip arg (=192.168.0.69) IP address of the sensor
  --xmlrpc-port arg (=80)  XMLRPC port of the sensor
  --password arg           Password for establishing an edit-session with the
                           sensor


These are common commands used in various situations:

    app-types     List the application types supported by the sensor.

    config        Configure sensor settings from a JSON description of
                  the desired sensor state. See also `dump'.

    cp            Create a new application on the sensor,
                  bootstrapped from a copy of an existing one.

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
accepts 1) a set of *global arguments* which control the particular camera you
wish to communicate with; 2) a subcommand; and 3) arguments to the
subcommand. To get a listing of the particular arguments accepted by a
subcommand, you can pass the `--help` option to the subcommand. For exemplary
purposes, let's list the options accepted by the `cp` subcommand.

```
$ ifm3d cp --help
usage: ifm3d [<global options>] cp [<cp options>]

global options:
  -h [ --help ]            Produce this help message and exit
  --ip arg (=192.168.0.69) IP address of the sensor
  --xmlrpc-port arg (=80)  XMLRPC port of the sensor
  --password arg           Password for establishing an edit-session with the
                           sensor

cp options:
  --index arg (=-1)     Index of source application to copy
```

As is shown above, `cp` takes a source application index to copy from.

We now walk through a couple of simple examples of using `ifm3d`. This is not an
exhaustive tutorial on `ifm3d` but rather intended to give a sense of how to
use the tool. The concepts apply broadly to all of the subcommands.

Example: Creating new applications
----------------------------------

Let's first list the applications on the camera.

```
$ ifm3d ls
[
  {
    "Active": true,
    "Description": "",
    "Id": 476707713,
    "Index": 1,
    "Name": "Sample Application"
  }
]
```

Now, let's create a new application whose settings will be bootstrapped from
the application at index `1`.

```
$ ifm3d cp --index=1
$ ifm3d ls
[
  {
    "Active": true,
    "Description": "",
    "Id": 476707713,
    "Index": 1,
    "Name": "Sample Application"
  },
  {
    "Active": false,
    "Description": "",
    "Id": 476707714,
    "Index": 2,
    "Name": "Sample Application"
  }
]
```

Now, let's create a new application from scratch (bootstrapped with
camera-default settings).

```
$ echo '{"Apps":[{}]}' | ifm3d config
$ ifm3d ls
[
  {
    "Active": true,
    "Description": "",
    "Id": 476707713,
    "Index": 1,
    "Name": "Sample Application"
  },
  {
    "Active": false,
    "Description": "",
    "Id": 476707714,
    "Index": 2,
    "Name": "Sample Application"
  },
  {
    "Active": false,
    "Description": "",
    "Id": 1755226334,
    "Index": 3,
    "Name": "New application"
  }
]
```

Now, let's set the application at index `3` to be the current active
application.

```
$ ifm3d dump | jq '.ifm3d.Device.ActiveApplication="3"' | ifm3d config
$ $ ifm3d ls
[
  {
    "Active": false,
    "Description": "",
    "Id": 476707713,
    "Index": 1,
    "Name": "Sample Application"
  },
  {
    "Active": false,
    "Description": "",
    "Id": 476707714,
    "Index": 2,
    "Name": "Sample Application"
  },
  {
    "Active": true,
    "Description": "",
    "Id": 1755226334,
    "Index": 3,
    "Name": "New application"
  }
]
```

Now, let's delete applications `2` and `3`.

```
$ ifm3d rm --index=2
$ ifm3d rm --index=3
$ ifm3d ls
[
  {
    "Active": false,
    "Description": "",
    "Id": 476707713,
    "Index": 1,
    "Name": "Sample Application"
  }
]
```

We note that based on the sequence of steps we took in this example, we are
currently left with a camera with only a single application but it is not
marked as active. So, let's set that application as "active" and validate it.

```
$ ifm3d dump | jq '.ifm3d.Device.ActiveApplication="1"' | ifm3d config
$ ifm3d ls
[
  {
    "Active": true,
    "Description": "",
    "Id": 476707713,
    "Index": 1,
    "Name": "Sample Application"
  }
]
```

Example: Setting the time on the camera
---------------------------------------

To set the time on the camera we use the `time` subcommand. Let's look at its
usage.

```
$ ifm3d time --help
usage: ifm3d [<global options>] time [<time options>]

global options:
  -h [ --help ]            Produce this help message and exit
  --ip arg (=192.168.0.69) IP address of the sensor
  --xmlrpc-port arg (=80)  XMLRPC port of the sensor
  --password arg           Password for establishing an edit-session with the
                           sensor

time options:
  --epoch arg           Secs since Unix epoch encoding time to be set on camera
                        (-1 == now)
```

To simply see the current time on the camera, we can issue the `time`
subcommand with no arguments.

```
$ ifm3d time
Local time on camera is: Tue Mar 13 18:22:16 2018
```

Let's now look at our local Unix time:

```
$ date
Mon May  7 16:20:49 EDT 2018
```

To synchronize the camera to our local time we can issue the following command.

```
$ ifm3d time --epoch=-1
Local time on camera is: Mon May  7 16:21:44 2018
```

And, double checking...

```
$ ifm3d time
Local time on camera is: Mon May  7 16:22:09 2018
```

(More examples to follow)
