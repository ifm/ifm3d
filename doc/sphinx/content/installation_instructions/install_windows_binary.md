## Install using the ifm3d installer

The instructions below show how to install the ifm3d library for C++ development and usage of the command line interface. We also provide a Python package `ifm3dpy`, see details [here](ifm3d/doc/sphinx/content/installation_instructions/install_py:Python%20installation).

### Installation

Download the ifm3d installer ifm3d_windows_x.x.x.exe from [ifm3d Release](https://github.com/ifm/ifm3d/releases). 
This installer provides binaries for Windows OS for Visual Studio 2019 and above.
For any other compiler we recommend to build ifm3d from sources.  
Follow the instructions on the installer.
It will install binaries at default location as ```C:/ProgramFiles/ifm3d x.x.x.```

### Usage

To use the ifm3d library in your own projects, make an environment variable with this path.
```bash
$ set IFM3D_BINARY_DIR = C:/ProgramFiles/ifm3d x.x.x/bin 
```

### Running ifm3d command line tools

After installing `ifm3d`, the binary files will be installed at
``%IFM3D_BINARY_DIR%\install\bin``. To run the ifm3d tool you need to add this
directory to your path.

If built targeting Visual Studio 2017/2019:
```bash
$ set PATH=%IFM3D_BINARY_DIR%;%PATH%
```

After that you should be able to run the ifm3d tool
```bash 
$ ifm3d
```

### Using ifm3d-playground projects

After installing the ifm3d, one can use ifm3d-playground example, which shows basic CMake configuration required for using 
installed ifm3d libraries [ifm3d playground example](https://github.com/ifm/ifm3d/tree/main/examples/o3r/ifm3d_playground)