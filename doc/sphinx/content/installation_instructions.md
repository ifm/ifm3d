
# Installing the software 

## O3R early adopters

### Linux
#### Source build
At this stage of the development, we require ifm3d to be installed from source.
Have a look at our instructions: {doc}`source_build`
### Docker dev containers
Development containers are available. They are built nightly with the latest version of ifm3d available on the o3r/main-next branch. You can pull them using the following command:
```
$ docker pull ghcr.io/ifm/ifm3d:latest
```

## Stable release

Binaries for ifm3d are available on a few supported platforms. Instructions for
each now follow.

### Linux

#### Ubuntu Linux via Apt (amd64/arm64)

⚠️ The provided apt repositories are experimental and shall be used with caution, the version uploaded to the apt repository might change and thus may break your use-case. If you rely on a specific version of the software we do recommend to run your own apt repository or build from source.

We provide apt repositories for the following Ubuntu Linux distributions and
architectures:

<table>
  <tr>
    <th/>
    <th>Ubuntu 16.04 Xenial</th>
    <th>Ubuntu 18.04 Melodic</th>
    <th>Ubuntu 20.04 Focal</th>
  </tr>
  <tr>
    <th>amd64</th>
    <td>X</td>
    <td>X</td>
    <td>X</td>
  </tr>
  <tr>
    <th>arm64</th>
    <td>X</td>
    <td>X</td>
    <td>X</td>
  </tr>
</table>

Add the repository to your sources.list:
```
$ sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] https://nexus.ifm.com/repository/ifm-robotics_ubuntu_$(lsb_release -sc)_$(dpkg --print-architecture) $(lsb_release -sc) main" > /etc/apt/sources.list.d/ifm-robotics.list'
```

Add the public key for the repository:
```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 8AB59D3A2BD7B692
```

If you experience issues with connecting the key server you can try this alternative which uses curl. This is maybe helpful when you are behind a proxyserver.
```
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0x8AB59D3A2BD7B692' | sudo apt-key add -
```
:exclamation: In case of any name resolution issues, it is worth to check the environment variable ```$https_proxy``` for proper proxy configuration.

Install the software:

```
$ sudo apt-get update
$ sudo apt-get install ifm3d-camera \
                       ifm3d-framegrabber \
                       ifm3d-swupdater \
                       ifm3d-image \
                       ifm3d-opencv \
                       ifm3d-pcicclient \
                       ifm3d-tools \
                       ifm3d-python3 \
                       ifm3d-pcl-viewer \
```

#### Linux for Tegra

⚠️ The provided apt repositories are experimental and shall be used with caution, the version uploaded to the apt repository might change and thus may break your use-case. If you rely on a specific version of the software we do recommend to run your own apt repository or build from source.

Linux for Tegra is an NVIDIA Linux distribution for the Jetson family of GPU
SoC systems. NVIDIA distributes a software package called JetPack with various
utilities and libraries optimized for the target hardware. There are a few
packages which override the core Ubuntu packages (`OpenCV` as the primary
example). We provide alternate apt repositories for `ifm3d` built on top of
the JetPack libraries rather than the Ubuntu libraries.

Add *one* of the following repositories based on your desired JetPack/L4T
release:

Jetpack 4.4:
```
$ sudo sh -c 'echo "deb https://nexus.ifm.com/repository/ifm-robotics_l4t_jetpack_4_4_arm64 melodic main" > /etc/apt/sources.list.d/ifm-robotics.list'
```
Jetpack 4.3:
```
$ sudo sh -c 'echo "deb https://nexus.ifm.com/repository/ifm-robotics_l4t_jetpack_4_3_arm64 melodic main" > /etc/apt/sources.list.d/ifm-robotics.list'
```

Add the public key for the repository:
```
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 8AB59D3A2BD7B692
```

Install the software:

```
$ sudo apt-get update
$ sudo apt-get install ifm3d-camera \
                       ifm3d-framegrabber \
                       ifm3d-swupdater \
                       ifm3d-image \
                       ifm3d-opencv \
                       ifm3d-pcicclient \
                       ifm3d-tools \
                       ifm3d-python3 \
                       ifm3d-pcl-viewer \
```

#### ROS/ROS2

For users interested in using our [ROS](https://github.com/ifm/ifm3d-ros)
bindings, `ifm3d` and `ifm3d-ros` are both available in the ROS distribution
for Kinetic and Melodic (Noetic coming shortly).
```
$ sudo apt install ros-kinetic-ifm3d
```
or
```
$ sudo apt install ros-melodic-ifm3d
```

For users interested in using our [ROS2](https://github.com/ifm/ifm3d-ros2)
bindings, binaries will be included (starting with dashing) very soon. For now,
packages must be built from source. Do not use the debian mirror for
`ifm3d` since (depending on version) ROS2 ships parallel versions of some core
libraries (OpenCV, PCL) as compared with standard Ubuntu. `ifm3d` must be built
against the proper dependencies.

### Windows

Pre-built binaries are not yet available. For now we recommend manually
compiling for the target MSVC/SDK versions according to the
building on {doc}`windows` instructions.


### Other platforms

If you are simply
looking to do a quick evaluation of an `ifm3d` supported sensor we recommend
the use of {doc}`../docker/README`.

Alternatively, you can build from source: {doc}`source_build`.

