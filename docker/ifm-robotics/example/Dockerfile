FROM ghcr.io/ifm/ifm3d:v1.3.3-ubuntu-amd64

RUN sudo apt update && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y\
        build-essential \
        cmake \
        coreutils 

COPY examples/o3r/ifm3d_playground /home/ifm/ifm3d_playground

WORKDIR /home/ifm/ifm3d_playground/build

RUN cmake .. && cmake --build .