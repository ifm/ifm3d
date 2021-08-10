#!/bin/bash

sudo apt-get update
sudo apt-get install -y --no-install-recommends \
       libboost-all-dev \
       git \
       openssh-client \
       libcurl4-openssl-dev \
       libgtest-dev \
       libgoogle-glog-dev \
       libxmlrpc-c++8-dev \
       libopencv-dev \
       libpcl-dev \
       libproj-dev \
       python3-dev \
       python3-pip \
       build-essential \
       coreutils \
       findutils \
       cmake \
       ninja-build \
       dpkg-dev \
       locales && \
       rm -rf /var/lib/apt/lists/*


# Update PIP this maybe required for some packages
python3 -m pip install -U pip

# Install missing Python packages
pip3 install -r requirements.txt


