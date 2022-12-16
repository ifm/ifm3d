# Set the base image
ARG BASE_IMAGE=ubuntu:20.04

# the cmake version to use for building
ARG CMAKE_VERSION=3.20.6

# This stage used as a base for building and the final image
FROM ${BASE_IMAGE} AS base

# Create the ifm user
RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm

# This stage is used for building ifm3d
FROM base AS build
ARG CMAKE_VERSION

# Install build-time dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        build-essential \
        cmake \
        coreutils \
        findutils \
        git \
        locales \
        ninja-build \
        libgtest-dev \
        libssl-dev \
        wget \
    && rm -rf /var/lib/apt/lists/*

# Install cmake 
RUN wget -O - "https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-$(uname -i).tar.gz" \
    | tar -xz --strip-components=1 -C /usr


# if you are running unit tests against a device at
# a different IP, set that here.
ENV IFM3D_IP 192.168.0.69

# Copy the code
COPY . /ifm3d

# Run the build
RUN mkdir -p /ifm3d/build \
    && cd /ifm3d/build \
    && cmake -GNinja \ 
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DBUILD_TESTS=ON \
        .. \
    && cmake --build . \
    && ninja check

ENTRYPOINT ["/bin/bash"]
