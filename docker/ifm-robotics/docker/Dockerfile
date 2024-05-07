# Set the base image
ARG BASE_IMAGE=ubuntu:20.04

# the cmake version to use for building
ARG CMAKE_VERSION=3.20.6

# if defined, we run unit tests when building ifm3d
ARG RUN_TESTS


# This stage used as a base for building and the final image
FROM ${BASE_IMAGE} AS base
ARG BASE_IMAGE

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

# Create the final image based on the base stage
FROM base
ARG BASE_IMAGE

# Install runtime requirements
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        libopencv-core* \
        libxmlrpc-c++8v5 \
        locales \
        sudo \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /home/ifm/tmp

COPY ./ifm3d*.tar /home/ifm/tmp/

RUN BASE_IMAGE_NAME=$(echo "${BASE_IMAGE}" | grep -o '[^/]*$' | cut -d":" -f1) \
    && BASE_IMAGE_VERSION=$(echo "${BASE_IMAGE}" | grep -o '[^/]*$' | cut -d":" -f2) \
    && ARCH=$(dpkg --print-architecture) \
    && tarname=ifm3d-$BASE_IMAGE_NAME-$BASE_IMAGE_VERSION-$ARCH-debs \
    && tarfile=$(find /home/ifm/tmp -type f -name "${tarname}*" -print -quit) \
    && tar -xv -C /home/ifm/tmp -f "$tarfile" \
    && dpkg -i /home/ifm/tmp/*.deb \
    && rm -rf /home/ifm/tmp

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