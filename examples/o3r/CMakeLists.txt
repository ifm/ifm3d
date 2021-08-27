cmake_minimum_required(VERSION 3.16)

##############################################################
## finding the ifm3d lib.
##############################################################
find_package(ifm3d CONFIG
  REQUIRED COMPONENTS camera framegrabber image
  )


##############################################################
## Building the examples
##############################################################

## Configuration example
add_executable(configuration_ex tutorials/configuration/configuration_ex.cpp)
target_link_libraries(configuration_ex
                      ifm3d::camera
                      )
set_target_properties(configuration_ex
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples/o3r/tutorials/configuration"
)                      

## Getting data example
add_executable(getting_data_ex tutorials/gettingData/getting_data_ex.cpp)
target_link_libraries(getting_data_ex
                      ifm3d::camera
                      ifm3d::framegrabber
                      ifm3d::image
                      )
set_target_properties(getting_data_ex
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples/o3r/tutorials/gettingData"
)


## Multi head example
add_executable(multi_head_ex tutorials/multiHead/multi_head_ex.cpp)
target_link_libraries(multi_head_ex
                      ifm3d::camera
                      ifm3d::framegrabber
                      ifm3d::image
                      )
set_target_properties(multi_head_ex
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples/o3r/tutorials/multiHead"
)
