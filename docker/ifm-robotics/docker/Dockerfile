# Set the base image
ARG BASE_IMAGE=ubuntu:20.04

# Set the Python version to use
ARG PYTHON_VERSION=3.13

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
        build-essential \
        gnupg2 \
        wget \
        curl \
        libssl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN echo "Setting up Uv package manager"
# Install Uv package manager
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# Ensure Uv's bin directory is in PATH
ENV PATH="/root/.local/bin:$PATH"

# Install Python 3.13 using uv
RUN uv python install ${PYTHON_VERSION}

# Create a virtual environment with python 3.13
RUN uv venv --python="$(echo ${PYTHON_VERSION})" --seed /home/ifm/venv

# Upgrade pip, setuptools, and wheel
RUN /home/ifm/venv/bin/python -m pip install --upgrade pip setuptools wheel

# Add the virtual environment's bin directory to PATH
ENV PATH="/home/ifm/venv/bin:$PATH"

# Create the ifm user
RUN id ifm 2>/dev/null || useradd --uid 30000 --create-home -s /bin/bash -U ifm

# Create the final image based on the base stage
FROM base
ARG BASE_IMAGE

# Install runtime requirements
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        locales \
        sudo \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /home/ifm/tmp

COPY ./ifm3d*.deb /home/ifm/tmp/
RUN BASE_IMAGE_NAME=$(echo "${BASE_IMAGE}" | grep -o '[^/]*$' | cut -d":" -f1) \
    && BASE_IMAGE_VERSION=$(echo "${BASE_IMAGE}" | grep -o '[^/]*$' | cut -d":" -f2) \
    && ARCH=$(dpkg --print-architecture) \
    && cd /home/ifm/tmp/ \
    && dpkg -i ifm3d-${BASE_IMAGE_NAME}-${BASE_IMAGE_VERSION}-${ARCH}-*.deb \
    # && dpkg -i ifm3dpy-${BASE_IMAGE_NAME}-${BASE_IMAGE_VERSION}-${ARCH}-*.deb \
    && rm -rf /home/ifm/tmp

# Copy wheels from the build context to the Docker image
RUN echo "Copying wheels from ./filtered_wheels"
COPY ./filtered_wheels /filtered_wheels

# Install wheel
RUN /home/ifm/venv/bin/pip install /filtered_wheels/*.whl 

# Setup localisation
RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    /usr/sbin/update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN echo "ifm ALL=(ALL) NOPASSWD: ALL" | tee /etc/sudoers.d/ifm

USER ifm

ENTRYPOINT ["/bin/bash"]