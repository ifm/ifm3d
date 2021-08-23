# Python version
ARG PYTHON_VERSION=3.9

# Set the base image
ARG BASE_IMAGE=ubuntu:20.04

# if defined, we run unit tests when building ifm3d
ARG RUN_TESTS

FROM ${BASE_IMAGE} AS base
ARG PYTHON_VERSION

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y gnupg2 \
    && UBUNTU_RELEASE=$(awk -F'=' '/UBUNTU_CODENAME/ {print $2}' /etc/os-release) \
    && echo "deb http://ppa.launchpad.net/deadsnakes/ppa/ubuntu ${UBUNTU_RELEASE} main" > "/etc/apt/sources.list.d/deadsnakes-ubuntu-ppa-${UBUNTU_RELEASE}.list" \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F23C5A6CF475977595C89F51BA6932366A755776 \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        python${PYTHON_VERSION} \
        python${PYTHON_VERSION}-venv \
    && rm -rf /var/lib/apt/lists/*

RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm

RUN python${PYTHON_VERSION} -m venv /home/ifm/venv \
    && . /home/ifm/venv/bin/activate \
    && pip install --upgrade \
        numpy \
        pip \
        pytest \
        setuptools \
        wheel \
    && chown -R ifm:ifm /home/ifm/venv


FROM base AS build
ARG PYTHON_VERSION

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
        python${PYTHON_VERSION}-dev \
    && rm -rf /var/lib/apt/lists/*

# build pybind11 with cmake - but first clone from the official github repo
RUN git clone --branch v2.3.0 https://github.com/pybind/pybind11.git /pybind  \
    && mkdir -p /pybind/build \
    && cd /pybind/build \
    && cmake -GNinja \
        -DPYBIND11_TEST=OFF \
        -DCMAKE_INSTALL_PREFIX=/app \
        -DPYTHON_EXECUTABLE=/usr/bin/python${PYTHON_VERSION} \
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
        -DCMAKE_INSTALL_PREFIX=/app \
        -DBUILD_MODULE_OPENCV=ON \
        -DBUILD_MODULE_PCICCLIENT=ON \
        -DBUILD_MODULE_PYBIND11=ON \
        -DPYTHON_EXECUTABLE=/usr/bin/python${PYTHON_VERSION} \
        .. \
    && cmake --build . \
    && if [ "x$RUN_TESTS" = "x" ]; then echo "Skipping tests..."; else ninja check; fi \
    && cmake --build . --target install

# multistage to reduce image size
FROM base
ARG PYTHON_VERSION

COPY --from=build /app /usr

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libgoogle-glog0v5 \
        libopencv-core* \
        libxmlrpc-c++8v5 \
        sudo \
    && rm -rf /var/lib/apt/lists/*

RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    /usr/sbin/update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN echo "ifm ALL=(ALL) NOPASSWD: ALL" | tee -a /etc/sudoers

USER ifm
ENV VIRTUAL_ENV="/home/ifm/venv"
ENV PATH="/home/ifm/venv/bin:$PATH"

ENTRYPOINT ["/bin/bash"]