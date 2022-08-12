# cmake_minimum_required(VERSION 3.16)

##############################################################
## Building the examples
##############################################################

## Configuration example
add_executable(configuration configuration/configuration.cpp)
target_link_libraries(configuration
                      ifm3d_device
                      )

## Getting data example
add_executable(getting_data getting_data/getting_data.cpp)
target_link_libraries(getting_data
                      ifm3d_device
                      ifm3d_framegrabber
                      )


## Multi head example
add_executable(multi_head multi_head/multi_head.cpp)
target_link_libraries(multi_head
                      ifm3d_device
                      ifm3d_framegrabber
                      )

# Heads "sync" example
find_package(fmt)
add_executable(time_sync time_sync/time_sync.cpp)
target_link_libraries(time_sync
                      ifm3d_device
                      ifm3d_framegrabber
                      fmt::fmt
                      )
                                            
## ifm3d-example playground 
add_executable(ifm3d_playground ifm3d_playground/ifm3d_playground.cpp)
target_link_libraries(ifm3d_playground
                      ifm3d_device
                      ifm3d_framegrabber
                      )
