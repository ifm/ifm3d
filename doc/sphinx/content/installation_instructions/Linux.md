## Installing ifm3d from debs file 

**Note**: If you are using O3D/O3X device plese use ifm3d-v0.20.3 Release and use These [instructions](https://github.com/ifm/ifm3d/blob/legacy/doc/source_build.md)

### Download the .debs files

Download the ifm3d debs files for you OS ifm3d-ubuntu-<ubuntu_version>_<arch>_debs.xz.tar From [ifm3d Release page](https://github.com/ifm/ifm3d/releases).
This debs files are ifm3d modules deps without its dependencies.

### Install the ifm3d third-party dependencies

Use the following steps to install all the library dependencies on Debian based systems
  
```
$ sudo apt-get update && sudo apt-get -y upgrade
$ sudo apt-get update && sudo apt-get install -y
      jq \ 
      libssl-dev \
      libcurl4-openssl-dev \
      libgoogle-glog-dev  \
      libxmlrpc-c++8-dev \ 
      libproj-dev \
      build-essential \
      coreutils \
      cmake                         
```

Note: The package name may differ in different flavours of Linux. 
Above apt-get commands are specific to Debian based systems

### Install ifm3d .debs files 
unzip the .tar file and use following command to install ifm3d from deps

```
$ sudo dpkg -i ifm3d_X.X.X_amd64-device.deb
$ sudo dpkg -i ifm3d_1.18.0_amd64-swupdater.deb
$ sudo dpkg -i ifm3d_1.18.0_amd64-framegrabber.deb
$ sudo dpkg -i ifm3d_1.18.0_amd64-tools.deb
```

This will install the required binaries and headers to /usr folder.
