
ifm3d - Inspecting and configuring the O3X camera/imager settings
=================================================================

`ifm3d` provides access to the camera/imager parameters via the `ifm3d dump` and
`ifm3d config` command line programs. The approach employed by `ifm3d`
is to encode the settings via a JSON serialization. To inpsect the settings you
use `ifm3d dump` to get a JSON encoding of the parameters. To mutate the
settings you use `ifm3d config` by providing a JSON serialization of your
*desired* settings. Examples on how to use these services now follows.

Lets first consider an O3X in a factory default state. In the code block below,
we first factory-default our camera, then dump its settings:

```
    $ ifm3d reset

    $ ifm3d dump
{
  "ifm3d": {
    "Apps": [
      {
        "Description": "",
        "Id": "1034160761",
        "Imager": {
          "ExposureTime": "1000",
          "FrameRate": "5",
          "MaxAllowedFrameRate": "12.5",
          "MinimumAmplitude": "42",
          "SpatialFilter": {
            "MaskSize": "0"
          },
          "SpatialFilterType": "1",
          "SymmetryThreshold": "0.4",
          "TemporalFilter": {},
          "TemporalFilterType": "0",
          "Type": "UR03.10m_1FRQ_1EXP_0GRAY"
        },
        "Index": "1",
        "Name": "",
        "OutputAmplitudeImage": "true",
        "OutputConfidenceImage": "false",
        "OutputDistanceImage": "true",
        "OutputGrayscaleImage": "false",
        "OutputXYZImage": "false",
        "TriggerMode": "1",
        "Type": "Camera"
      }
    ],
    "Device": {
      "ArticleNumber": "M03457",
      "ArticleStatus": "AA",
      "Description": "",
      "DeviceType": "1:512",
      "IPAddressConfig": "0",
      "ImageTimestampReference": "1495614156",
      "Name": "New sensor",
      "OperatingMode": "0",
      "PasswordActivated": "false",
      "SessionTimeout": "30",
      "UpTime": "0.136944444444444"
    },
    "Net": {
      "MACAddress": "00:02:01:40:54:09",
      "NetworkSpeed": "0",
      "StaticIPv4Address": "192.168.0.69",
      "StaticIPv4Gateway": "192.168.0.201",
      "StaticIPv4SubNetMask": "255.255.255.0",
      "UseDHCP": "false"
    },
    "Time": {
      "CurrentTime": "1495614156",
      "NTPServers": "",
      "StartingSynchronization": "false",
      "Stats": "",
      "SynchronizationActivated": "false",
      "Syncing": "false",
      "WaitSyncTries": "1"
    },
    "_": {
      "Date": "Thu Sep 14 08:54:32 2017",
      "HWInfo": {
        "MACAddress": "00:02:01:40:54:09",
        "Mainboard": "#!03_M100_B01_12345678_008025483",
        "MiraSerial": "Not implemented"
      },
      "SWVersion": {
        "Algorithm_Version": "0.1.5",
        "Calibration_Device": "ff:ff:ff:ff:ff:ff",
        "Calibration_Version": "99.99.99",
        "ELDK": "GOLDENEYE_YOCTO_HARDFP-303-06d9c894636352a6c93711c7284d02b0c794a527",
        "IFM_Software": "0.1.20",
        "Linux": "Linux version 3.14.34-rt31-yocto-standard-00016-g5121435-dirty (jenkins@dettlx152) (gcc version 4.9.2 (GCC) ) #1 SMP PREEMPT RT Tue Mar 14 08:40:14 CET 2017",
        "Main_Application": "0.4.1122"
      },
      "ifm3d_version": 302
    }
  }
}
```

Let's say that we now wanted to change the imager to employ two frequencies (to
extend our range) and two exposure times (to increase the fidelity of our depth
data). The first thing we need to do is understand the special strings used to
spell out the imager name. To get a list of available imager types we can use
the following command:

```
    $ ifm3d imager-types
[
  "UR03.10m_1FRQ_1EXP_0GRAY",
  "UR03.10m_1FRQ_2EXP_0GRAY",
  "UR15.50m_FRAC8,7_2FRQ_1EXP_0GRAY",
  "UR15.50m_FRAC8,7_2FRQ_2EXP_0GRAY"
]
```

We see the last one includes the strings `2FRQ` and `2EXP`, this is what we
want. To set that on the camera, we can use the following command:

```
    $ echo '{"Apps":[{"Index":"1","Imager":{"Type":"UR15.50m_FRAC8,7_2FRQ_2EXP_0GRAY"}}]}' | ifm3d config
```

We can now check that our settings have been applied with the following command
(this assumes you have the json *grepping* tool `jq` installed. If you do not,
you can simply inspect the whole dump):

```
    $ ifm3d dump | jq .ifm3d.Apps[0].Imager.Type
"UR15.50m_FRAC8,7_2FRQ_2EXP_0GRAY"
```

Now, let's look again at our factory defaulted imager settings:

```
$ ifm3d dump | jq .ifm3d.Apps[0].Imager
{
  "ExposureTime": "1000",
  "ExposureTimeRatio": "40",
  "FrameRate": "5",
  "MaxAllowedFrameRate": "6.09756097560976",
  "MinimumAmplitude": "42",
  "SpatialFilter": {
    "MaskSize": "0"
  },
  "SpatialFilterType": "1",
  "SymmetryThreshold": "0.4",
  "TemporalFilter": {},
  "TemporalFilterType": "0",
  "Type": "UR15.50m_FRAC8,7_2FRQ_2EXP_0GRAY"
}
```

We can see that there is (by default) a spatial filter applied
(`SpatialFilterType`) and it has a mask parameter (`MaskSize`). This happens to
be a spatial median filter with a 3x3 mask. These special codes are defined in
the [camera.h](https://github.com/ifm/ifm3d/blob/master/modules/camera/include/ifm3d/camera/camera.h)
header file. So, assuming you have consulted that file where the constants are
defined, we could set our median filter to a 5x5 mask with the following:

```
    $ echo '{"Apps":[{"Index":"1","Imager":{"SpatialFilter":{"MaskSize":"1"}}}]}' | ifm3d config
```

And we can check that the settings were accepted by the camera:

```
$ ifm3d dump | jq .ifm3d.Apps[0].Imager
{
  "ExposureTime": "1000",
  "ExposureTimeRatio": "40",
  "FrameRate": "5",
  "MaxAllowedFrameRate": "6.09756097560976",
  "MinimumAmplitude": "42",
  "SpatialFilter": {
    "MaskSize": "1"
  },
  "SpatialFilterType": "1",
  "SymmetryThreshold": "0.4",
  "TemporalFilter": {},
  "TemporalFilterType": "0",
  "Type": "UR15.50m_FRAC8,7_2FRQ_2EXP_0GRAY"
}
```

From the above description, it would follow that to turn off spatial filtering,
you could use the following command:

```
    $ echo '{"Apps":[{"Index":"1","Imager":{"SpatialFilterType":"0"}}]}' | ifm3d config
```

We can validate our settings:

```
    $ ifm3d dump | jq .ifm3d.Apps[0].Imager
{
  "ExposureTime": "1000",
  "ExposureTimeRatio": "40",
  "FrameRate": "5",
  "MaxAllowedFrameRate": "6.09756097560976",
  "MinimumAmplitude": "42",
  "SpatialFilter": {},
  "SpatialFilterType": "0",
  "SymmetryThreshold": "0.4",
  "TemporalFilter": {},
  "TemporalFilterType": "0",
  "Type": "UR15.50m_FRAC8,7_2FRQ_2EXP_0GRAY"
}
```

While full documentation on all imager parameters are forthcoming, this
document should serve as a quick-start reference to access and mutate camera
parameters from the command-line using the `ifm3d` tool.
