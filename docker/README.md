ifm3d - Docker
==============

NOTE: This work with Docker is currently experimental. The process will be
refined over time and is subject to change without notice. Thanks for your
patience.

As of ifm3d version 0.12.0, we are using [Docker](https://www.docker.com/) as a
means to build binary packages of the software, for Linux, to help ease
installation for our users. We use Docker in two ways:

1. To build the software packages (deb files)
2. To provide a quick and easy way to test those debs on a fresh OS
   installation.

Whether you plan to use Docker for building, running, or both, you will need to
install Docker. There are two steps to getting Docker setup correctly:

1. Install Docker using
   [these instructions](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
2. Carry out the post installation setup using
   [these instructions](https://docs.docker.com/install/linux/linux-postinstall/)


Building
--------

This section is only for ifm3d developers. Users can safely skip this part.

To use Docker to build the ifm3d software packages, you should utilize the
[build-debs.py](build/build-debs.py) script (requires Python 3). The help
output for the script is as follows:

```
$ ./build-debs.py --help
usage: build-debs.py [-h] [--docker_file DOCKER_FILE [DOCKER_FILE ...]]
                     [--image IMAGE] [--vtag VTAG] [--run_tests]

Build ifm3d debs via Docker

optional arguments:
  -h, --help            show this help message and exit
  --docker_file DOCKER_FILE [DOCKER_FILE ...]
                        DockerFile list to process. If not specified, all
                        `DockerFile' specifications encountered in all
                        subirectories (with respect to your current working
                        directory), recursively, will be processed.
  --image IMAGE         Docker image name.
  --vtag VTAG           ifm3d git version tag to build. If omitted, `master'
                        will be checked out and built.
  --run_tests           Run unit tests during build. Only specify this flag if
                        testing against live hardware (e.g., O3D or O3X).
```

Depending upon how you parameterize `build-debs.py`, the built packages will be
placed next to the `Dockerfile` in a subdirectory called `debs`. These debs can
then be uploaded to ifm's Nexus server for easy install via `apt-get`.

Running
-------

If you just want to quickly evaluate an ifm 3D camera via `ifm3d`, you can
bring up a Docker container appropriate to your situation. We maintain a few
to make this process easy.

### Non-ROS users

Dockerfiles to build can be found [here](run/amd64/ubuntu).

__Ubuntu 16.04__

Be sure you are in the [16.04 directory](run/amd64/ubuntu/16.04).

Build the Docker container:

```
$ docker build -t ifm3d1604 .
```

Once the container is done being built, if you simply want to inspect your
connected camera via the `ifm3d` command line tool, you can:

```
$ docker run -u ifm -ti --rm ifm3d1604:latest /bin/bash
ifm@c8c9865efb14:/$ ifm3d ls
[
  {
    "Active": true,
    "Description": "",
    "Id": 351437496,
    "Index": 1,
    "Name": "Sample Application"
  }
]
ifm@c8c9865efb14:/$ ifm3d hz
FrameGrabber running at: 4.95815 Hz
10 frames captured, over 1 runs
ifm@c8c9865efb14:/$ ^D
```

If you want to visualize the data using the
[ifm3d-pcl-viewer](https://github.com/ifm/ifm3d-pcl-viewer):

NOTE: The `xhost` command below is considered *insecure*. For exemplary
purposes, we use this approach in these instructions. However, we urge you to
read the `man` page for `xhost` and decide if this is a security concern for
you before running the commands below. You have been warned.

```
$ xhost +local:root
non-network local connections being added to access control list

$ docker run -u ifm -ti --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ifm3d1604:latest /bin/bash
ifm@b79de0545832:/$ ifm3d-pcl-viewer
```

If your camera is connected at the default ip address of 192.168.0.69, you
should now see a window rendering the streaming pointcloud.

__Ubuntu 18.04__

Be sure you are in the [18.04 directory](run/amd64/ubuntu/18.04).

Build the Docker container:

```
$ docker build -t ifm3d1804 .
```

Use the command line tools:

```
$ docker run -u ifm -ti --rm ifm3d1804:latest /bin/bash
ifm@330fc18969c4:/$ ifm3d ls
[
  {
    "Active": true,
    "Description": "",
    "Id": 351437496,
    "Index": 1,
    "Name": "Sample Application"
  }
]
ifm@330fc18969c4:/$ ifm3d hz
FrameGrabber running at: 5.33508 Hz
10 frames captured, over 1 runs
ifm@330fc18969c4:/$ ^D
```

Use the [visualizer](https://github.com/ifm/ifm3d-pcl-viewer):

```
$ xhost +local:root
non-network local connections being added to access control list

$ docker run -u ifm -ti --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ifm3d1804:latest /bin/bash
ifm@f98b659d1e71:/$ ifm3d-pcl-viewer
```

__Ubuntu 20.04__

Be sure you are in the [20.04 directory](run/amd64/ubuntu/20.04).

Build the Docker container:

```
$ docker build -t ifm3d2004 .
```

Use the command line tools:

```
$ docker run -u ifm -ti --rm ifm3d2004:latest /bin/bash
ifm@330fc18969c4:/$ ifm3d ls
[
  {
    "Active": true,
    "Description": "",
    "Id": 351437496,
    "Index": 1,
    "Name": "Sample Application"
  }
]
ifm@330fc18969c4:/$ ifm3d hz
FrameGrabber running at: 5.33508 Hz
10 frames captured, over 1 runs
ifm@330fc18969c4:/$ ^D
```

Use the [visualizer](https://github.com/ifm/ifm3d-pcl-viewer):

```
$ xhost +local:root
non-network local connections being added to access control list

$ docker run -u ifm -ti --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw ifm3d2004:latest /bin/bash
ifm@f98b659d1e71:/$ ifm3d-pcl-viewer
```