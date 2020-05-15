#
# Toolchain for cross-building imf3d for the O3D3xx camera.
#
# NOTE: For this file to work, you must first:
# source /opt/poky/1.8.1/environment-setup-armv7ahf-vfp-neon-poky-linux-gnueabi
#

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSROOT /opt/poky/1.8.1/sysroots/armv7ahf-vfp-neon-poky-linux-gnueabi)
set(SYSROOT_CONTRIB /opt/poky/contrib)
set(CMAKE_FIND_ROOT_PATH ${SYSROOT_CONTRIB} ${CMAKE_SYSROOT})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_LIBRARY_PATH /lib
                       /usr/lib
                       /usr/lib/arm-linux-gnueabihf)

set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
