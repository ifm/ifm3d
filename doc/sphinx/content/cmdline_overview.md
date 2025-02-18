
# ifm3d - Command Line Tool

## Overview

`ifm3d` ships with a command line tool of the same name. The `ifm3d` command
line tool is used to both introspect the state of a camera as well as mutate
parameters. 
To carry out a particular task, you evoke one of the `ifm3d`
*subcommands*. To get a listing of available subcommands, you can pass the
`--help` option.

```
$ ifm3d --help
ifm3d: version=1.6.6

TODO FILL OUT HERE
```

Each of the supported family of cameras, O3X1xx, O3D3xx and OVP8xx, provides specific subcommands that cater to their specific features. 
To access these, the name of the camera family will be used as the first subcommand, followed by the command of interest. 

For example, to read out the current configuration on a connected OVP8xx, one would use:
```
ifm3d ovp8xx config get

TODO FILL OUT HERE
```

When accessing a device using non-default parameters, one can add global arguments to the command. 
For example, to read out the current configuration of a connected VPU, at IP 192.168.0.70, one would use:
```
ifm3d ovp8xx config get --ip=192.168.0.70

TODO FILL OUT HERE
```

The documentation for each command and subcommand is available using the `--help` argument. 
For example, to display the documentation for the O3D3xx TODO TODO

## Configuring

Using the ifm3d CLI can be a quick way to configure or read out parts of the device configuration.
The configuration uses the JSON format, which means that the command-line JSON processor `jq` can be used to further process the results (TODO is this still needed now?).

For example, to change the name of an OVP8xx device, one can use:
```
ifm3d ovp8xx config set TODO
```

Alternatively, a configuration file can be used, by providing its path in the `--file` argument.
For example:
```
ifm3d ovp8xx config set --file=TODO
```