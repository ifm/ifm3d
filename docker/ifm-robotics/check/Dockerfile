ARG BASE_IMAGE=mcr.microsoft.com/devcontainers/cpp:1-ubuntu-24.04
ARG COMMAND=clang-format

FROM ${BASE_IMAGE}
ARG BASE_IMAGE

COPY scripts /tmp/setup/

RUN bash /tmp/setup/setup-dev-container.sh && rm -rf /tmp/setup

RUN mkdir -p /ifm3d
WORKDIR /ifm3d

ENTRYPOINT [ "/bin/bash" ]
