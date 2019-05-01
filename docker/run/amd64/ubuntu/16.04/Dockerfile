FROM ubuntu:16.04

#####################################################
#####################################################

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get -y upgrade
RUN apt-get update && \
    apt-get install -y jq \
                       locales \
                       curl \
                       sudo \
                       apt-transport-https \
                       mesa-utils

RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    /usr/sbin/update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN echo 'deb [arch=amd64] https://nexus.ifm.com/repository/ifm-robotics_ubuntu_xenial_amd64 xenial main' > /etc/apt/sources.list.d/ifm-robotics.list
RUN apt-key adv \
         --keyserver hkp://ha.pool.sks-keyservers.net:80 \
         --recv-key 8AB59D3A2BD7B692
RUN apt-get update && \
    apt-get install -y ifm3d-camera \
                       ifm3d-framegrabber \
                       ifm3d-image \
                       ifm3d-opencv \
                       ifm3d-pcicclient \
                       ifm3d-tools \
                       ifm3d-pcl-viewer \
                       ifm3d-python \
                       ifm3d-python3

RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm
RUN echo "ifm ALL=(ALL) NOPASSWD: ALL" | tee -a /etc/sudoers
