## Ubuntu Linux via Apt (amd64/arm64)

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
