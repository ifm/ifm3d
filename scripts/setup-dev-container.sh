#!/bin/bash

set -e

################## Change these variables to update versions ###################

UBUNTU_CODENAME="noble"
CMAKE_VERSION="4.0.2"
LLVM_VERSION="20.1.8"

################################################################################

if [ "${UBUNTU_CODENAME}" != $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") ]; then
    echo "The current OS is not supported.";
    exit 1;
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Check if the script is run as root
if [ "$EUID" -ne 0 ]; then
    SUDO="sudo"
fi

$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
    build-essential \
    clang-format-14 \
    curl \
    coreutils \
    cppcheck \
    doxygen \
    dpkg-dev \
    findutils \
    git \
    graphviz \
    gpg \
    gpg-agent \
    jq \
    libcurl4-openssl-dev \
    libgtest-dev \
    locales \
    ninja-build \
    openssh-client \
    python3-dev \
    python3-pip \
    python3-venv \
    wget

# Install sccache if cargo is available
if command -v cargo &> /dev/null; then
    if [ -z "$CARGO_HOME" ]; then
        CARGO_HOME="${HOME}/.cargo"
    fi

    $SUDO mkdir -p ${CARGO_HOME}
    $SUDO chown -R $(whoami) ${CARGO_HOME}

    curl -L --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.sh | bash
    cargo binstall -y sccache
fi


# Setup python
$SUDO mkdir -p /opt/venv
$SUDO chown -R $(whoami) /opt/venv

PYTHON_VERSION_MAJOR=$(python3 --version | awk '{print $2}' | cut -d. -f1)
PYTHON_VERSION_MINOR=$(python3 --version | awk '{print $2}' | cut -d. -f2)

python3 -m venv /opt/venv
source /opt/venv/bin/activate

python -m pip install -U pip

if [ -f requirements.txt ]; then
    pip install -r requirements.txt
fi

# Create symlinks for the python module and stubs
ln -s /workspace/build/modules/pybind11/src/ifm3dpy.cpython-${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR}-x86_64-linux-gnu.so /opt/venv/lib64/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages/
ln -s /workspace/build/modules/pybind11/src/ifm3dpy /opt/venv/lib64/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages/

# Install cmake
$SUDO bash ${SCRIPT_DIR}/reinstall-cmake.sh ${CMAKE_VERSION}

# Install LLVM
$SUDO bash ${SCRIPT_DIR}/install-llvm.sh ${LLVM_VERSION}

# Install ctcache
$SUDO wget https://raw.githubusercontent.com/matus-chochlik/ctcache/b54f74807fc02c8897247fda6229aabbac78a560/src/ctcache/clang_tidy_cache.py -O /usr/local/bin/ctcache
$SUDO chmod +x /usr/local/bin/ctcache
pip install boto3