
ifm3d - Camera and Imager Configuration
=======================================

Configuring the parameters of an ifm 3D camera is accomplished in ifm3d in one
of two ways: 1) via the `ifm3d` command line tool; 2) via the `ifm3d`
library's `camera` module API. This document focuses on the `ifm3d` command
line tool. An ever-growing repository of API usage examples can be found
[here](https://github.com/ifm/ifm3d-examples).

The primary mechanism for using the `ifm3d` command line tool to configure an
ifm 3D camera is to utilize the `dump` and `config` subcommands to `ifm3d`. The
`dump` command serializes the camera state to JSON and prints it to `stdout`,
while the `config` subcommand consumes (either from a file or `stdin`) the same
JSON serialization but interprets that JSON as a desired state for the
camera. Said another way, `ifm3d config` will make a best effort attempt to
have the actual camera hardware reflect the camera state encoded in the JSON
stream passed to it.

The remainder of this document will contain a set of examples and associated
narrative in hopes of demonstrating how to leverage `ifm3d` to configure your
3D camera. For puproses of this document, an ifm O3D303 will be
utilized. However, the techniques shown here apply to any supported `ifm3d`
camera (e.g., O3X). Additionally, since the camera state is serialized via
JSON, some of the examples below will utilize the
[jq](https://stedolan.github.io/jq/) command line JSON processor to build up
Linux pipelines to carry out a specific task. The usage of `jq` is *not*
required. Standard Linux tools (`grep`, `sed`, `awk`, `perl`, `python`, etc.)
could also be used or a single pipeline can be decomposed into multiple
commands whereby data are serialized to a file, edited, then I/O redirected
into `ifm3d config` in discrete steps. Again, the remainder of this document
will assume `jq` is available. (To install `jq` on Ubuntu:
`sudo apt-get install jq`).

Dump
----

Serializing the current state of the camera is accomplished through the `ifm3d
dump` command. Exemplary output for an O3D303 is shown below.

```
$ ifm3d dump
{
  "ifm3d": {
    "Apps": [
      {
        "Description": "",
        "Id": "476707713",
        "Imager": {
          "AutoExposureMaxExposureTime": "10000",
          "AutoExposureReferencePointX": "88",
          "AutoExposureReferencePointY": "66",
          "AutoExposureReferenceROI": "{\"ROIs\":[{\"id\":0,\"group\":0,\"type\":\"Rect\",\"width\":130,\"height\":100,\"angle\":0,\"center_x\":88,\"center_y\":66}]}",
          "AutoExposureReferenceType": "0",
          "Channel": "0",
          "ClippingBottom": "131",
          "ClippingCuboid": "{\"XMin\": -3.402823e+38, \"XMax\": 3.402823e+38, \"YMin\": -3.402823e+38, \"YMax\": 3.402823e+38, \"ZMin\": -3.402823e+38, \"ZMax\": 3.402823e+38}",
          "ClippingLeft": "0",
          "ClippingRight": "175",
          "ClippingTop": "0",
          "ContinuousAutoExposure": "false",
          "EnableAmplitudeCorrection": "true",
          "EnableFastFrequency": "false",
          "EnableFilterAmplitudeImage": "true",
          "EnableFilterDistanceImage": "true",
          "EnableRectificationAmplitudeImage": "false",
          "EnableRectificationDistanceImage": "false",
          "ExposureTime": "5000",
          "ExposureTimeList": "125;5000",
          "ExposureTimeRatio": "40",
          "FrameRate": "5",
          "MaxAllowedLEDFrameRate": "23.2",
          "MinimumAmplitude": "42",
          "Resolution": "0",
          "SpatialFilter": {},
          "SpatialFilterType": "0",
          "SymmetryThreshold": "0.4",
          "TemporalFilter": {},
          "TemporalFilterType": "0",
          "ThreeFreqMax2FLineDistPercentage": "80",
          "ThreeFreqMax3FLineDistPercentage": "80",
          "TwoFreqMaxLineDistPercentage": "80",
          "Type": "under5m_moderate",
          "UseSimpleBinning": "false"
        },
        "Index": "1",
        "LogicGraph": "{\"IOMap\": {\"OUT1\": \"RFT\",\"OUT2\": \"AQUFIN\"},\"blocks\": {\"B00001\": {\"pos\": {\"x\": 200,\"y\": 200},\"properties\": {},\"type\": \"PIN_EVENT_IMAGE_ACQUISITION_FINISHED\"},\"B00002\": {\"pos\": {\"x\": 200,\"y\": 75},\"properties\": {},\"type\": \"PIN_EVENT_READY_FOR_TRIGGER\"},\"B00003\": {\"pos\": {\"x\": 600,\"y\": 75},\"properties\": {\"pulse_duration\": 0},\"type\": \"DIGITAL_OUT1\"},\"B00005\": {\"pos\": {\"x\": 600,\"y\": 200},\"properties\": {\"pulse_duration\": 0},\"type\": \"DIGITAL_OUT2\"}},\"connectors\": {\"C00000\": {\"dst\": \"B00003\",\"dstEP\": 0,\"src\": \"B00002\",\"srcEP\": 0},\"C00001\": {\"dst\": \"B00005\",\"dstEP\": 0,\"src\": \"B00001\",\"srcEP\": 0}}}",
        "Name": "Sample Application",
        "PcicEipResultSchema": "{ \"layouter\": \"flexible\", \"format\": { \"dataencoding\": \"binary\", \"order\": \"big\" }, \"elements\" : [ { \"type\": \"string\", \"value\": \"star\", \"id\": \"start_string\" }, { \"type\": \"records\", \"id\": \"models\", \"elements\": [ { \"type\": \"int16\", \"id\": \"boxFound\" }, { \"type\": \"int16\", \"id\": \"width\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"height\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"length\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"xMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"zMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yawAngle\" }, { \"type\": \"int16\", \"id\": \"qualityLength\" }, { \"type\": \"int16\", \"id\": \"qualityWidth\" }, { \"type\": \"int16\", \"id\": \"qualityHeight\" } ] }, { \"type\": \"string\", \"value\": \"stop\", \"id\": \"end_string\" } ] }",
        "PcicPnioResultSchema": "{\"layouter\" : \"flexible\", \"format\": { \"dataencoding\": \"binary\", \"order\": \"big\" }, \"elements\" : [ { \"type\": \"string\", \"value\": \"star\", \"id\": \"start_string\" }, { \"type\": \"records\", \"id\": \"models\", \"elements\": [ { \"type\": \"int16\", \"id\": \"boxFound\" }, { \"type\": \"int16\", \"id\": \"width\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"height\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"length\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"xMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"zMidTop\", \"format\": { \"scale\": 1000 } }, { \"type\": \"int16\", \"id\": \"yawAngle\" }, { \"type\": \"int16\", \"id\": \"qualityLength\" }, { \"type\": \"int16\", \"id\": \"qualityWidth\" }, { \"type\": \"int16\", \"id\": \"qualityHeight\" } ] }, { \"type\": \"string\", \"value\": \"stop\", \"id\": \"end_string\" } ] }",
        "PcicTcpResultSchema": "{ \"layouter\": \"flexible\", \"format\": { \"dataencoding\": \"ascii\" }, \"elements\": [ { \"type\": \"string\", \"value\": \"star\", \"id\": \"start_string\" }, { \"type\": \"blob\", \"id\": \"normalized_amplitude_image\" }, { \"type\": \"blob\", \"id\": \"distance_image\" }, { \"type\": \"blob\", \"id\": \"x_image\" }, { \"type\": \"blob\", \"id\": \"y_image\" }, { \"type\": \"blob\", \"id\": \"z_image\" }, { \"type\": \"blob\", \"id\": \"confidence_image\" }, { \"type\": \"blob\", \"id\": \"diagnostic_data\" }, { \"type\": \"string\", \"value\": \"stop\", \"id\": \"end_string\" } ] }",
        "TemplateInfo": "",
        "TriggerMode": "1",
        "Type": "Camera"
      }
    ],
    "Device": {
      "ActiveApplication": "1",
      "ArticleNumber": "O3D303",
      "ArticleStatus": "AD",
      "Description": "",
      "DeviceType": "1:2",
      "EnableAcquisitionFinishedPCIC": "false",
      "EthernetFieldBus": "0",
      "EthernetFieldBusEndianness": "0",
      "EvaluationFinishedMinHoldTime": "10",
      "ExtrinsicCalibRotX": "0",
      "ExtrinsicCalibRotY": "0",
      "ExtrinsicCalibRotZ": "0",
      "ExtrinsicCalibTransX": "0",
      "ExtrinsicCalibTransY": "0",
      "ExtrinsicCalibTransZ": "0",
      "IODebouncing": "true",
      "IOExternApplicationSwitch": "0",
      "IOLogicType": "1",
      "IPAddressConfig": "0",
      "ImageTimestampReference": "1520963818",
      "Name": "New sensor",
      "OperatingMode": "0",
      "PNIODeviceName": "",
      "PasswordActivated": "false",
      "PcicProtocolVersion": "3",
      "PcicTcpPort": "50010",
      "SaveRestoreStatsOnApplSwitch": "true",
      "ServiceReportFailedBuffer": "15",
      "ServiceReportPassedBuffer": "15",
      "SessionTimeout": "30",
      "TemperatureFront1": "3276.7",
      "TemperatureFront2": "3276.7",
      "TemperatureIMX6": "40.7179985046387",
      "TemperatureIllu": "47.5",
      "UpTime": "1.61972222222222"
    },
    "Net": {
      "MACAddress": "00:02:01:40:7D:96",
      "NetworkSpeed": "0",
      "StaticIPv4Address": "192.168.0.69",
      "StaticIPv4Gateway": "192.168.0.201",
      "StaticIPv4SubNetMask": "255.255.255.0",
      "UseDHCP": "false"
    },
    "Time": {
      "CurrentTime": "1520963816",
      "NTPServers": "",
      "StartingSynchronization": "false",
      "Stats": "",
      "SynchronizationActivated": "false",
      "Syncing": "false",
      "WaitSyncTries": "2"
    },
    "_": {
      "Date": "Mon May  7 11:55:08 2018",
      "HWInfo": {
        "Connector": "#!02_A300_C40_02452814_008023176",
        "Diagnose": "#!02_D322_C32_03038544_008023267",
        "Frontend": "#!02_F342_C34_17_00049_008023607",
        "Illumination": "#!02_I300_001_03106810_008001175",
        "MACAddress": "00:02:01:40:7D:96",
        "Mainboard": "#!02_M381_003_03181504_008022954",
        "MiraSerial": "0e30-59af-0ef7-0244"
      },
      "SWVersion": {
        "Algorithm_Version": "2.0.45",
        "Calibration_Device": "00:02:01:40:7d:96",
        "Calibration_Version": "0.9.0",
        "Diagnostic_Controller": "v1.0.69-9dbc4ca5ef-dirty",
        "ELDK": "GOLDENEYE_YOCTO/releases%2FO3D%2FRB_1.20.x-7-06d9c894636352a6c93711c7284d02b0c794a527",
        "IFM_Recovery": "unversioned",
        "IFM_Software": "1.20.1138",
        "Linux": "Linux version 3.14.34-rt31-yocto-standard-00009-ge4ab4d94f288-dirty (jenkins@dettlx190) (gcc version 4.9.2 (GCC) ) #1 SMP PREEMPT RT Tue Mar 13 16:06:07 CET 2018",
        "Main_Application": "unknown"
      },
      "ifm3d_version": 900
    }
  }
}
```

In the example above, we serialize the entire state of the camera. This is
useful to, for example, save to a file, edit, and push out to one or more
cameras. However, sometimes we need to look at the camera configuration to
simply answer a question we may have. For example, if we wanted to see which
version of the firmware the camera is running we could issue the following
command.

```
$ ifm3d dump | jq .ifm3d._.SWVersion.IFM_Software
"1.20.1138"
```

It follows that the entire JSON serialized configuration may be further
processed either programmatically or manually via a text editor.


Config
------

Mutating parameters on the camera is done by creating a *desired* camera state
encoded in JSON compliant to the output produced by `ifm3d dump`. For example,
if we wanted to change the framerate of the *first* application on the camera
we could do the following.

1. Before changing the framerate, let's see what it is currently set to:

(NOTE: This step is not necessary. We do this to illustrate the state of the
camera prior to mutating the parameter).

```
$ ifm3d dump | jq .ifm3d.Apps[0].Imager.FrameRate
"5"
```

We see the current framerate is 5 fps.

2. Let's set it to 10 fps:

```
$ ifm3d dump | jq '.ifm3d.Apps[0].Imager.FrameRate="10"' | ifm3d config
```

3. Let's check to make sure that our configuration has persisted.

```
$ ifm3d dump | jq .ifm3d.Apps[0].Imager.FrameRate
"10"
```

Let's now break down what we did in this single Linux pipeline.

```
$ ifm3d dump | jq '.ifm3d.Apps[0].Imager.FrameRate="10"' | ifm3d config
```

First we dump the entire state of the camera to JSON, process the JSON in-line
via `jq` to set the FrameRate to 10 fps, then pipe the resulting output to
`ifm3d config` which accepts the new (mutated) JSON stream on `stdin` and
carries out the necessary network calls to mutate the camera settings and
persist them.

This is the basic paradigm that can be followed to tune just about any
parameter on the camera. To carry out more complex configuration tasks (e.g.,
changing several parameters at once), the dump can be saved to a file, edited
via a text editor, then fed into `ifm3d config` to perform the
configuration. It is also important to point out that `ifm3d config` does not
need the entire ifm3d JSON *object* to operate correctly. *Snippets* are
valid. For example, if we wanted to set the framerate back to `5`, we could do
this:

```
$ echo '{"Apps":[{"Index":"1","Imager":{"FrameRate":"5"}}]}' | ifm3d config
```

Let's validate that it worked:

```
$ ifm3d dump | jq .ifm3d.Apps[0].Imager.FrameRate
"5"
```

In summary, the primary concept in configuring your camera via `ifm3d` is that
the `dump` subcommand can be used to access the current camera state while the
`config` subcommand is used to declare and mutate the camera into a desired
state -- assuming the desired state is valid.

If there are questions, please post them to our
[issue tracker](https://github.com/ifm/ifm3d/issues).
