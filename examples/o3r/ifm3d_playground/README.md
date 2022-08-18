# ifm3d playground project

```ifm3d_playground``` provides the out of ifm3d source example with cmake configurations.
User can copy ifm3d_playground folder out of the ifm3d source and can use following instructions
to build the example and develop it further.

```bash
$ cd ifm3d_playground
$ mkdir build 
$ cd build 
$ cmake -DCMAKE_PREFIX_PATH=<ifm3d install path> ..
$ cmake --build . --config Release --target ALL_BUILD
```