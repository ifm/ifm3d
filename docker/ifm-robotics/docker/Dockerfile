# Set the base image
ARG BASE_IMAGE=ubuntu:20.04

# the cmake version to use for building
ARG CMAKE_VERSION=3.20.6

# if defined, we run unit tests when building ifm3d
ARG RUN_TESTS


# This stage used as a base for building and the final image
FROM ${BASE_IMAGE} AS base

# install python
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        gnupg2 \
        wget \
        python3 \
        python3-pip \
        python3-venv \
        python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Create the ifm user
RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm

# Create a venv and install python dependencies
RUN python3 -m venv /home/ifm/venv \
    && . /home/ifm/venv/bin/activate \
    && python -m pip install --upgrade pip \
    && python -m pip install --upgrade \
        numpy \
        pytest \
        setuptools \
        wheel


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
        libboost-all-dev \
        libcurl4-openssl-dev \
        libgtest-dev \
        libopencv-dev \
        libpcl-dev \
        libproj-dev \
        libxmlrpc-c++8-dev \
        locales \
        ninja-build \
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
        -DCMAKE_INSTALL_PREFIX=/app \
        -DPYTHON_EXECUTABLE=/home/ifm/venv/bin/python3 \
        .. \
    && cmake --build . \
    && cmake --build . --target install

# if you are running unit tests against a camera at
# a different IP, set that here.
ENV IFM3D_IP 192.168.0.69

# Copy the code
COPY . /ifm3d

# Run the build
RUN mkdir -p /ifm3d/build \
    && cd /ifm3d/build \
    && cmake -GNinja \ 
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/app \
        -DBUILD_MODULE_DESERIALIZE=ON \
        -DBUILD_MODULE_OPENCV=ON \
        -DBUILD_MODULE_PCICCLIENT=ON \
        -DBUILD_MODULE_PYBIND11=ON \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        .. \
    && cmake --build . \
    && if [ "x$RUN_TESTS" = "x" ]; then echo "Skipping tests..."; else ninja check; fi \
    && cmake --build . --target install \
    && cp /ifm3d/build/modules/pybind11/src/ifm3dpy.*.so /home/ifm/venv/lib/python*/site-packages/

# Create the final image based on the base stage
FROM base

# Copy the build output
COPY --from=build /app /usr
COPY --from=build --chown=ifm:ifm /home/ifm/venv/ /home/ifm/venv/

# Install runtime requirements
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libopencv-core* \
        libxmlrpc-c++8v5 \
        locales \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Setup localisation
RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    /usr/sbin/update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN echo "ifm ALL=(ALL) NOPASSWD: ALL" | tee /etc/sudoers.d/ifm

USER ifm
ENV VIRTUAL_ENV="/home/ifm/venv"
ENV PATH="/home/ifm/venv/bin:$PATH"

ENTRYPOINT ["/bin/bash"]