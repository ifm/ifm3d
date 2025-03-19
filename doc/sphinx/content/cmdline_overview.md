
# ifm3d command line tool

## Overview

`ifm3d` ships with a command line tool of the same name. The `ifm3d` command
line tool is used to both introspect the state of a camera as well as mutate
parameters. 
To carry out a particular task, you evoke one of the `ifm3d`
*subcommands*. To get a listing of available subcommands, you can pass the
`--help` option.

```
$ ifm3d --help
ifm3d Command Line Interface (CLI)
Usage: ifm3d [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit
  -v,--version                Print version information
  --ip TEXT:IPV4 [192.168.0.69] 
                              IP address of the sensor
  --xmlrpc-port UINT [80]     XMLRPC port of the sensor
  --password TEXT             Password for establishing an edit-session with the sensor
  --log-level TEXT:{CRITICAL,ERROR,WARN,INFO,DEBUG,VERBOSE,NONE} [WARN] 
                              The minimum severity of messages that will be logged, possible values are in descending order
  --log-file TEXT             Log to a file instead of stderr

Subcommands:
  o3x1xx                      Interact with the O3X1xx devices
  o3d3xx                      Interact with the O3D3xx devices
  ovp8xx                      Interact with the OVP8xx video processing units
  discover                    Discover ifm devices on the network.
```

Each of the supported family of cameras, O3X1xx, O3D3xx and OVP8xx, provides specific subcommands that cater to their specific features. 
To access these, the name of the camera family will be used as the first subcommand, potentially followed by additional commands. 

For example, to read out the current configuration on a connected OVP8xx, one would use:
```
$ ifm3d ovp8xx config get
{
  "applications": {
    "classes": {
      "mcc": {
        "instanceSchema": {
          "$schema": "http://json-schema.org/draft-07/schema#",
          "additionalProperties": false,
          "properties": {
            "class": {},
            "configuration": {},
            "data": {},
...
```

When accessing a device using non-default parameters, one can add global arguments to the command. 
For example, to read out the current configuration of a connected OVP, at IP 192.168.0.70, one would use:
```
$ ifm3d ovp8xx config get --ip=192.168.0.70
{
  "applications": {
    "classes": {
      "mcc": {
        "instanceSchema": {
          "$schema": "http://json-schema.org/draft-07/schema#",
          "additionalProperties": false,
          "properties": {
            "class": {},
            "configuration": {},
            "data": {},
...
```

The documentation for each command and subcommand is available using the `--help` argument. 
For example, to display the documentation for the O3D3xx:
```
$ ifm3d o3d3xx --help
Interact with the O3D3xx devices
Usage: ifm3d o3d3xx [OPTIONS] SUBCOMMAND

Options:
  -h,--help                   Print this help message and exit

Subcommands:
  app-types                   List the application types supported by the sensor.
  config                      Configure sensor settings from a JSON description of the desired sensor state. See also 'dump'.
  cp                          Create a new application on the sensor, bootstrapped from a copy of an existing one.
  discover                    Discover ifm devices on the network.
  dump                        Serialize the sensor state to JSON.
  export                      Export an application or whole sensor configuration into a format compatible with ifm Vision Assistant.
  hz                          Compute the actual frequency at which the FrameGrabber is running.
  imager-types                List the imager types supported by the sensor.
  import                      Import an application or whole sensor configuration that is compatible with ifm Vision Assistant's export format (*.o3d3xxapp).
  jitter                      Collects statistics on framegrabber (and optionally, image construction) jitter.
  ls                          Lists the applications currently installed on the sensor.
  passwd                      Sets the password on the sensor.
  reboot                      Reboot the sensor
  reset                       Reset the sensor to factory defaults.
  rm                          Deletes an application from the sensor.
  swupdate                    Perform a firmware update on the camera. 
  time                        Get/set the current time on the camera.
  trace                       Get trace messages from the internal camera trace buffer.
```

## Configuring

Using the ifm3d CLI can be a quick way to configure or read out parts of the device configuration.
It can be used in conjunction with the tool [jq](https://jqlang.org/) to set portions of the configuration, or alternatively a configuration file in JSON format can be used.

For example, to change the name of an OVP8xx device, one can use:
```
$ echo {} | jq '.device.info.name="My favorite O3R"' | ifm3d ovp8xx config set
```

Alternatively, a configuration file can be used, by providing its path in the `--file` argument.
For example:
```
$ ifm3d ovp8xx config set --file=config.json
```

To read out a specific subset of the configuration, one can use:
```
$ ifm3d ovp8xx config get --path "/device/info/name"
{
  "device": {
    "info": {
      "name": "My favorite O3R"
    }
  }
}
```

:::{note}
The configuration uses the JSON format, which means that the command-line JSON processor `jq` can be used to further process the results.
:::

## Diagnostic

[![O3R](https://img.shields.io/badge/O3R-green.svg)]()
[![O3D](https://img.shields.io/badge/O3D-red.svg)]()
[![O3X](https://img.shields.io/badge/O3X-red.svg)]()

### Device diagnostic

The O3R platforms provides diagnostic to help monitor the health of the system at runtime, and to provide debug information in case troubleshooting help is needed.

To retrieve the currently active diagnostic, one can use:
```
$ ifm3d ovp8xx diagnostic get
{
  "bootid": "2e49ea23-2f03-4965-8d48-fe08e72c23dd",
  "events": [],
  "timestamp": "1651170509113082016",
  "version": {
    "diagnostics": "1.0.1",
    "euphrates": "1.42.9",
    "firmware": "1.20.11.6034"
  }
}
```
In the example above, there is no active diagnostics.

By default, only the active diagnostic is displayed. To filter the diagnostic messages, for example to retrieve all active diagnostics related to the application `app0`, use:
```
$ ifm3d ovp8xx diagnostic get --filter '{"source":"/applications/instances/app0", "state": "active"}'

TODO
```
### Service report
In addition to the diagnostic, a service report can be downloaded. 
The generated ZIP file should be shared with ifm support team when requesting troubleshooting help, as it contains detailed information about the state and configuration of the system.

To download it, use:
```
$ ifm3d ovp8xx getServiceReport > service_report.zip
$ unzip service_report.zip
$ ls
configuration.json  deserializer_info.json  diagnostics.json  emmc_devices_health.json  filesystem_info.txt  journalctl.log  manifest.json  schema.json  service_report.zip
```