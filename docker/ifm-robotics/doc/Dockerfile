ARG BASE_IMAGE=ubuntu:20.04

FROM ${BASE_IMAGE} AS build
ARG BASE_IMAGE

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        build-essential \
        cmake \
        coreutils \
        doxygen \
        findutils \
        git \
        graphviz \
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
        python3-sphinx \
        python3-sphinx-rtd-theme \
    && rm -rf /var/lib/apt/lists/*

# build pybind11 with cmake - but first clone from the official github repo
RUN git clone --branch v2.8.0 https://github.com/pybind/pybind11.git /pybind  \
    && mkdir -p /pybind/build \
    && cd /pybind/build \
    && cmake -GNinja \
        -DPYBIND11_TEST=OFF \
        -DCMAKE_INSTALL_PREFIX=/app \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        .. \
    && cmake --build . \
    && cmake --build . --target install

# if you are running unit tests against a camera at
# a different IP, set that here.
ENV IFM3D_IP 192.168.0.69

COPY . /ifm3d

RUN pip3 install -r /ifm3d/doc/sphinx/requirements.txt

RUN mkdir -p /ifm3d/build \
    && cd /ifm3d/build \
    && cmake -GNinja \ 
        -DCMAKE_INSTALL_PREFIX=/app \
        -DBUILD_IN_DEPS=OFF \
        -DBUILD_MODULE_IMAGE=ON \
        -DBUILD_MODULE_OPENCV=ON \
        -DBUILD_MODULE_PCICCLIENT=ON \
        -DBUILD_MODULE_PYBIND11=ON \
        -DBUILD_MODULE_TOOLS=ON \
        -DBUILD_DOC=ON \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        .. \
    && cmake --build . -j 4 \
    && if [ "x$RUN_TESTS" = "x" ]; then echo "Skipping tests..."; else ninja check; fi \
    && cmake --build . --target docs

FROM scratch AS output
COPY --from=build /ifm3d/build/doc/sphinx/html /out/html
COPY --from=build /ifm3d/build/doc/doxygen/latex/cpp_api /out/latex

