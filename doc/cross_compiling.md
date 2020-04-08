# Cross-compiling ifm3d

One benefit of the CMake build system is the possibility to cross compile your code. To demonstrate this with ifm3d we use a Toolchain created with OpenEmbedded as a showcase


# Installing a Toolchain

Depending on your Toolchain provider this step might look different

```
$ chmod +x poky-glibc-i686-vantage-image-armv7ahf-vfp-neon-toolchain-qte-1.8.1.sh
$ ./poky-glibc-i686-vantage-image-armv7ahf-vfp-neon-toolchain-qte-1.8.1.sh
```

# Using the Toolchain

```
$ git clone https://github.com/ifm/ifm3d.git
$ cd ifm3d/
$ mkdir build
$ source /opt/poky/1.8.1/environment-setup-armv7ahf-vfp-neon-poky-linux-gnueabi
$ cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchains/o3d303.cmake -DCMAKE_INSTALL_PREFIX=../../oem-partition/usr ..
```
The step ``source /opt/poky/1.8.1/environment-setup-armv7ahf-vfp-neon-poky-linux-gnueabi`` is necessary for the OpenEmbedded Toolchain to set all the needed Environment variables.

The variable ``CMAKE_INSTALL_PREFIX`` may get handy when you plan to install the ifm3d library and tools to a specific location for an OEM deployment for example.
To run the build you may have to adjust the argument of ``-j`` depending on the number of CPU cores available in your system

```
$ make -j8
$ make install
```

# Caution

Both the provided Toolchain file [cmake/toolchains/o3d303.cmake](/cmake/toolchains/o3d303.cmake) and the instructions above are examples for the OEM workflow. Both maybe needs adjustments if you are planning to use a different type of set-up

