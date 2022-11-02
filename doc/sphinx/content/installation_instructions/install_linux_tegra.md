
## Linux for Tegra

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
$ sudo apt-get install ifm3d-device \
                       ifm3d-framegrabber \
                       ifm3d-swupdater \
                       ifm3d-pcicclient \
                       ifm3d-tools \
                       ifm3d-python3 \
```
