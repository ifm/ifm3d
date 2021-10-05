# cmake_minimum_required(VERSION 3.16)

##############################################################
## Building the examples
##############################################################

## Configuration example
add_executable(configuration configuration/configuration.cpp)
target_link_libraries(configuration
                      ifm3d_camera
                      )

## Getting data example
add_executable(getting_data getting_data/getting_data.cpp)
target_link_libraries(getting_data
                      ifm3d_camera
                      ifm3d_framegrabber
                      ifm3d_stlimage
                      )


## Multi head example
add_executable(multi_head multi_head/multi_head.cpp)
target_link_libraries(multi_head
                      ifm3d_camera
                      ifm3d_framegrabber
                      ifm3d_stlimage
                      )