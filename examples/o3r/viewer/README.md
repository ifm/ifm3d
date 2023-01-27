# ifm3dpy Viewer

This is an example application for retrieving different kinds of images from an O3R platform.

## Download the code
If you built ifm3d from source, you already have the code, in ifm3d/examples/o3r/viewer. 
If not, you can find the script [here](https://github.com/ifm/ifm3d/tree/main/examples/o3r/viewer).
## Installation 

The recommended way is to use a venv. 

### Create a new venv

```sh
python3 -m venv my_venv
```

### Activate the venv

| Platform | Shell           | Command to activate virtual environment |
| -------- | --------------- | --------------------------------------- |
| POSIX    | bash/zsh        | $ source <venv>/bin/activate            |
|          | fish            | $ source <venv>/bin/activate.fish       |
|          | csh/tcsh        | $ source <venv>/bin/activate.csh        |
|          | PowerShell Core | $ <venv>/bin/Activate.ps1               |
| Windows  | cmd.exe         | C:\> <venv>\Scripts\activate.bat        |
|          | PowerShell      | PS C:\> <venv>\Scripts\Activate.ps1     |

### Install ifm3dpy
#### From pip
```sh
pip install ifm3dpy
```
#### From Source
```sh
# In the ifm3d root folder
pip install -r requirements.txt
pip install .
```
Consider [the python documentation](../../../doc/sphinx/content/python.md) for more details.

### Install requirements
```sh
pip install -r examples/python/viewer/requirements.txt
```

## Usage
```sh
usage: ifm3dpy_viewer.py [-h] --pcic-port PORT --image {jpeg,distance,amplitude,xyz} [--ip IP] [--xmlrpc-port XMLRPC_PORT]

optional arguments:
  -h, --help            show this help message and exit
  --pcic-port PORT      The pcic port from which images should be received
  --image {jpeg,distance,amplitude,xyz}
                        The image to received (default: distance)
  --ip IP               IP address of the sensor (default: 192.168.0.69)
  --xmlrpc-port XMLRPC_PORT
                        XMLRPC port of the sensor (default: 80)
```

### Display the distance image
```sh
python examples/python/viewer/ifm3dpy_viewer.py --pcic-port 50012 --image distance
```

### Display the amplitude image
```sh
python examples/python/viewer/ifm3dpy_viewer.py --pcic-port 50012 --image amplitude
```

### Display the point cloud
```sh
python examples/python/viewer/ifm3dpy_viewer.py --pcic-port 50012 --image xyz
```

### Display the jpeg image
```
python examples/python/viewer/ifm3dpy_viewer.py --pcic-port 50010 --image jpeg
```
