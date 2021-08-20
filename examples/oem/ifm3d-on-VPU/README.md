# Using the ifm3d locally on the VPU

It is possible to use the ifm3d library within a container, locally running on the VPU. To configure the VPU, or to receive data, etc. the `IP` parameter for the `ifm3d` need to be updated. To achieve this, the network of the host(VPU) needs to be activated for the container, running the `ifm3d`

## Starting the container with the network=host

It is possible to define the container network to use the host network. This is needed, because the loopback (127.0.0.1) address within the container, will not reach the host otherwise.

To use the host network, `docker run` needs the `--network host` information:

Without the host network:

```bash
o3r-vpu-c0:~$ docker run -it --rm nexus.ifm.com:20443/ifm-robotics:l4t-latest
ifm@d918bad5d03b:/$ ifm3d dump --ip 127.0.0.1
null
```

With the host network:

```bash
o3r-vpu-c0:~$ docker run -it --rm --network=host nexus.ifm.com:20443/ifm-robotics:l4t-latest
ifm@o3r-vpu-c0:/$ ifm3d dump --ip 127.0.0.1
{
  "device": {
```

`ifm3d dump` forwards the VPU configuration. Using `--ip 127.0.0.1` tells `ifm3d` to connect to a different Ip address, than the default: `IFM3D_IP=192.168.0.69`

## Use different environment variable for easier connection

To avoid the `--ip 127.0.0.1` parameter for the `ifm3d`, it is possible to set the `IFM3D_IP` variable, during the start process of the container.

*Note: This is only working, if either the `l4t-latest` base image from ifm is used, or the container was created with the `IFM3D_IP` as an environment variable within the Dockerfile.*

Use `-e` to provide the environment variable:

```bash
o3r-vpu-c0:~$ docker run -it --rm --network host -e "IFM3D_IP=127.0.0.1"  nexus.ifm.com:20443/ifm-robotics:l4t-latest
ifm@o3r-vpu-c0:/$ ifm3d dump
{
  "device": {
    "clock": {
```

*Note: The IP address cannot be provided with `"` or `'`. Example: `IFM3D_IP="127.0.0.1"` will fail.*

## User docker-compose for easier usage

Instead of providing the right arguments etc. via `docker run`, `docker-compose` can be used as an easier approach. The network and the environment variable can be provided via the `docker-compose.yml` file:

```yml
version: "3.2"
services:
  ifm3d:
    image: nexus.ifm.com:20443/ifm-robotics:l4t-latest
    network_mode: host
    environment:
      - IFM3D_IP=127.0.0.1
```

It is possible to start the service with `docker-compose up`. Use `docker-compose run ifm3d` - `ifm3d` being the service name provided by the `.yml` - to start the container interactively.