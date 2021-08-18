# See here for image contents: https://github.com/microsoft/vscode-dev-containers/tree/v0.191.0/containers/cpp/.devcontainer/base.Dockerfile

# [Choice] Debian / Ubuntu version: debian-11, debian-10, debian-9, ubuntu-20.04, ubuntu-18.04
ARG VARIANT="ubuntu-20.04"
FROM mcr.microsoft.com/vscode/devcontainers/cpp:0-${VARIANT}

# [Optional] Uncomment this section to install additional packages.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
     && apt-get -y install --no-install-recommends \
                    locales
# Locales set-up
RUN echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
     locale-gen en_US.UTF-8 && \
     /usr/sbin/update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8