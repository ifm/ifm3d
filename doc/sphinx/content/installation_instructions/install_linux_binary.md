## Installing ifm3d from .deb file

**Note**: If you are using O3D/O3X device please use ifm3d-v0.20.3 release and use these [instructions](https://github.com/ifm/ifm3d/blob/v0.20.3/doc/source_build.md)

### Download ifm3d .deb files

Download the ifm3d .deb files for your OS ifm3d-ubuntu-<ubuntu_version>_<arch>_debs.xz.tar from [ifm3d Release page](https://github.com/ifm/ifm3d/releases).
This .deb files are of ifm3d modules without their dependencies.

### Install ifm3d third-party dependencies

Use the following steps to install all the library dependencies on Debian based systems
  
```
$ sudo apt-get update && sudo apt-get -y upgrade
$ sudo apt-get update && sudo apt-get install -y
      jq \ 
      libssl-dev \
      build-essential \
      coreutils \
      cmake                         
```

Note: The package name may differ in different flavours of Linux. 
Above apt-get commands are specific to Debian based systems

### Install ifm3d .deb files 
Unzip the .tar file and use following command to install ifm3d from .deb:

```
$ sudo dpkg -i ifm3d_1.0.1_amd64-device.deb
$ sudo dpkg -i ifm3d_1.0.1_amd64-swupdater.deb
$ sudo dpkg -i ifm3d_1.0.1_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_1.0.1_amd64-tools.deb
```

One can install all the debs will following command:
```
$ cd <folder containing all the debs>
$ sudo dpkg -R -i .
```

This will install the required binaries and headers to `/usr` folder.
