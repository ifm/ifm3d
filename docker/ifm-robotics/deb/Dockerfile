ARG BASE_IMAGE=ubuntu:20.04

# the cmake version to use for building
ARG CMAKE_VERSION=3.20.6

FROM ${BASE_IMAGE} AS build
ARG BASE_IMAGE
ARG CMAKE_VERSION

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        build-essential \
        cmake \
        coreutils \
        findutils \
        git \
        libboost-all-dev \
        libcurl4-openssl-dev \
        libgoogle-glog-dev \
        libgtest-dev \
        libopencv-dev \
        libpcl-dev \
        libproj-dev \
        libxmlrpc-c++8-dev \
        locales \
        ninja-build \
        python3-dev \
        python3-numpy \
        python3-pip \
        python3-pytest \
        python3-setuptools \
        wget \
    && rm -rf /var/lib/apt/lists/*

# Install cmake 
RUN wget -O - "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-$(uname -i).tar.gz" \
    | tar -xz --strip-components=1 -C /usr

# build pybind11 with cmake - but first clone from the official github repo
RUN git clone --branch v2.8.0 https://github.com/pybind/pybind11.git /pybind  \
    && mkdir -p /pybind/build \
    && cd /pybind/build \
    && cmake -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DPYBIND11_TEST=OFF \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        .. \
    && cmake --build . \
    && cmake --build . --target install

# if you are running unit tests against a camera at
# a different IP, set that here.
ENV IFM3D_IP 192.168.0.69

COPY . /ifm3d

RUN mkdir -p /ifm3d/build \
    && cd /ifm3d/build \
    && cmake -GNinja \ 
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DBUILD_MODULE_OPENCV=ON \
        -DBUILD_MODULE_PCICCLIENT=ON \
        -DBUILD_MODULE_PYBIND11=ON \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DBUILD_IN_DEPS=OFF \
        .. \
    && cmake --build . \
    && if [ "x$RUN_TESTS" = "x" ]; then echo "Skipping tests..."; else ninja check; fi \
    && cmake --build . --target package \
    && IFM3D_VERSION=$(/ifm3d/build/modules/tools/src/bin/ifm3d --version | cut -d "=" -f 2) \
    && BASE_IMAGE_NAME=$(echo "${BASE_IMAGE}" | grep -o '[^/]*$' | cut -d":" -f1) \
    && BASE_IMAGE_VERSION=$(echo "${BASE_IMAGE}" | grep -o '[^/]*$' | cut -d":" -f2) \
    && ARCH=$(dpkg --print-architecture) \
    && mkdir -p /out \
    && cd /ifm3d/build/  \
    && mv /ifm3d/build/ifm3d*.deb /out/ifm3d-${BASE_IMAGE_NAME}-${BASE_IMAGE_VERSION}-${ARCH}-${IFM3D_VERSION}.deb

FROM scratch AS output
COPY --from=build /out /out

