## Examples

### Creating new applications
[![O3R](https://img.shields.io/badge/O3R-lightgrey.svg)]()
[![O3D](https://img.shields.io/badge/O3D-green.svg)]()
[![O3X](https://img.shields.io/badge/O3X-green.svg)]()

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

### Setting NTP-Server connection on the camera
[![O3R](https://img.shields.io/badge/O3R-lightgrey.svg)]()
[![O3D](https://img.shields.io/badge/O3D-green.svg)]()
[![O3X](https://img.shields.io/badge/O3X-green.svg)]()


Using `jq`, you can set easily the NTP-Server on a camera. You just need to provide the
right IP address. In this case, the IP: 192.168.0.100 is the NTP server.

```
ifm3d dump | jq '.ifm3d.Time.NTPServers="192.168.0.100"' | ifm3d config

```
After that, we need to activate the usage of the NTP server too.

```
ifm3d dump | jq '.ifm3d.Time.SynchronizationActivated="True"' | ifm3d config
```

### Setting the time on the camera
[![O3R](https://img.shields.io/badge/O3R-lightgrey.svg)]()
[![O3D](https://img.shields.io/badge/O3D-green.svg)]()
[![O3X](https://img.shields.io/badge/O3X-green.svg)]()

To set the time on the camera we use the `time` subcommand. Let's look at its
usage.

```
$ ifm3d time --help
Usage:
  ifm3d [<global options>] time [<time options>]

 global options:
  -h, --help             Produce this help message and exit
      --ip arg           IP address of the sensor (default: 192.168.0.69)
      --xmlrpc-port arg  XMLRPC port of the sensor (default: 80)
      --password arg     Password for establishing an edit-session with the
                         sensor (default: )

 time options:
      --epoch arg  Secs since Unix epoch encoding time to be set on camera
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
