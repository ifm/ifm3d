#!/usr/bin/env bash

set -e

if [ "$EUID" -ne 0 ]; then
    SUDO="sudo"
fi
    
if [ -z "$CARGO_HOME" ]; then
    CARGO_HOME="${HOME}/.cargo"
fi

$SUDO mkdir -p ${CARGO_HOME}
$SUDO chown -R $(whoami) ${CARGO_HOME}

curl -L --proto '=https' --tlsv1.2 -sSf https://raw.githubusercontent.com/cargo-bins/cargo-binstall/main/install-from-binstall-release.sh | bash
cargo binstall -y sccache