ARG BASE_IMAGE=ubuntu:20.04
ARG CLANG_FORMAT_VERSION=10

FROM ${BASE_IMAGE}
ARG BASE_IMAGE
ARG CLANG_FORMAT_VERSION

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        clang-format-${CLANG_FORMAT_VERSION} \
        python3 \
    && rm -rf /var/lib/apt/lists/*

COPY . /ifm3d

WORKDIR /ifm3d
RUN python3 clang_format.py check