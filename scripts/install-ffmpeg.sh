#!/usr/bin/env bash
#
# Build and install FFmpeg (headers + shared libraries) so the RTSP ffmpeg
# decoder can be compiled in build environments that do not ship FFmpeg
# development files, e.g. the manylinux2014 Python wheel builders (CentOS 7,
# which has no ffmpeg-devel).
#
# The ffmpeg decoder resolves libavcodec/libavutil at runtime via dlopen and
# does NOT link against them, so only the FFmpeg *headers* are actually needed
# at build time; the libraries built here are a convenient way to obtain a
# consistent, self-generated set of headers (including the generated
# avconfig.h/version.h). At runtime the decoder loads whichever FFmpeg the host
# provides, falling back to the built-in "null" decoder when none is present.
# Only the long-stable front fields of AVFrame/AVPacket are accessed, so the
# build-time header version need not match the runtime library version.
#
# Override IFM3D_FFMPEG_VERSION to build against a different header set.
set -euo pipefail

FFMPEG_VERSION="${IFM3D_FFMPEG_VERSION:-4.4.4}"
PREFIX="${IFM3D_FFMPEG_PREFIX:-/usr/local}"

workdir="$(mktemp -d)"
trap 'rm -rf "${workdir}"' EXIT
cd "${workdir}"

echo "Installing FFmpeg ${FFMPEG_VERSION} (shared, link-only) into ${PREFIX}"

curl -fsSL "https://ffmpeg.org/releases/ffmpeg-${FFMPEG_VERSION}.tar.xz" \
  -o ffmpeg.tar.xz
tar -xf ffmpeg.tar.xz
cd "ffmpeg-${FFMPEG_VERSION}"

# A deliberately minimal configuration: only the H.264 decoder is enabled and
# all assembly is disabled. This build is used solely to satisfy link-time
# symbols/headers, so its feature set and performance are irrelevant; the host's
# FFmpeg is used at runtime.
./configure \
  --prefix="${PREFIX}" \
  --enable-shared \
  --disable-static \
  --disable-programs \
  --disable-doc \
  --disable-asm \
  --disable-everything \
  --enable-decoder=h264 \
  --enable-parser=h264

make -j"$(nproc)"
make install

# Refresh the linker cache so the freshly installed libraries are discoverable.
ldconfig || true
