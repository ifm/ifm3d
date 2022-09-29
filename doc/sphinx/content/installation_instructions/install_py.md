## Python installation
> Note: We recommend for testing purposes to install the ifm3dpy package in an clean python environment first. You can use `python -m venv "venv-name"` to create a new virtual environment.

You can use the official PyPI package to install the ifm3dpy within your virtual environment:

```python
pip install ifm3dpy
```
Now, you can check your installation.

### Check the ifm3dpy installation

Let's verify quickly that the installation worked! This command should display the list of packages installed in your environment:

```python
pip freeze
```

Open up a python shell with:

```python
python.exe
OR
./python.exe
OR
python
```

Then try importing the package:

```python
import ifm3dpy
print(ifm3dpy.__version__)
>>>0.91.0
```

You can test the connection from VPU to camera head with following lines:

```python
from ifm3dpy import O3R
o3r = O3R()
config = o3r.get() #get the configuration saved on the VPU
```

Using the package `json` provides an easier tool for displaying JSON-Strings. The configuration from the VPU is always a JSON-String (output below shortened for display purposes).

```python
import json
print(json.dumps(config, indent=4))
>>>{
    "device": {
        "clock": {
            "currentTime": 1581090739817663072
        },
        "diagnostic": {
            "temperatures": [],
            "upTime": 94000000000
        },
        "info": {
            "device": "0301",
            "deviceTreeBinaryBlob": "tegra186-quill-p3310-1000-c03-00-base.dtb",
            "features": {},
            "name": "TableTop2",
            "partNumber": "M03975",
            "productionState": "AA",
            "serialNumber": "000201234176",
            "vendor": "0001"
        },
        "network": {
            "authorized_keys": "",
            "ipAddressConfig": 0,
            "macEth0": "00:04:4B:EA:9F:D1",
            "macEth1": "00:02:01:23:41:76",
            "networkSpeed": 1000,
            "staticIPv4Address": "192.168.0.69",
            "staticIPv4Gateway": "192.168.0.201",
            "staticIPv4SubNetMask": "255.255.255.0",
            "useDHCP": false
        },
        "state": {
            "errorMessage": "",
            "errorNumber": ""
        },
        "swVersion": {
            "kernel": "4.9.140-l4t-r32.4+gc35f5eb9d1d9",
            "l4t": "r32.4.3",
            "os": "0.13.13-221",
            "schema": "v0.1.0",
            "swu": "0.15.12"
        }
    },
    "ports": {
        "port0": {
            "acquisition": {
                "framerate": 10.0,
                "version": {
                    "major": 0,
                    "minor": 0,
                    "patch": 0
                }
            },
            "data": {
                "algoDebugConfig": {},
                "availablePCICOutput": [],
                "pcicTCPPort": 50010
            },
            "info": {
                "device": "2301",
                "deviceTreeBinaryBlobOverlay": "001-ov9782.dtbo",
                "features": {
                    "fov": {
                        "horizontal": 127,
                        "vertical": 80
                    },
                    "resolution": {
                        "height": 800,
                        "width": 1280
                    },
                    "type": "2D"
                },
                "name": "",
                "partNumber": "M03976",
                "productionState": "AA",
                "sensor": "OV9782",
                "sensorID": "OV9782_127x80_noIllu_Csample",
                "serialNumber": "000000000281",
                "vendor": "0001"
            },
            "mode": "experimental_autoexposure2D",
            "processing": {
                "extrinsicHeadToUser": {
                    "rotX": 0.0,
                    "rotY": 0.0,
                    "rotZ": 0.0,
                    "transX": 0.0,
                    "transY": 0.0,
                    "transZ": 0.0
                },
                "version": {
                    "major": 0,
                    "minor": 0,
                    "patch": 0
                }
            },
            "state": "RUN"
        },
        ...
}
```