# cmake_minimum_required(VERSION 3.16)

##############################################################
## Building the examples
##############################################################

## Configuration example
add_executable(configuration_ex configuration/configuration_ex.cpp)
target_link_libraries(configuration_ex
                      ifm3d_camera
                      )
set_target_properties(configuration_ex
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples/cpp/configuration"
)                      

## Getting data example
add_executable(getting_data_ex gettingData/getting_data_ex.cpp)
target_link_libraries(getting_data_ex
                      ifm3d_camera
                      ifm3d_framegrabber
                      ifm3d_stlimage
                      )
set_target_properties(getting_data_ex
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples/cpp/gettingData"
)


## Multi head example
add_executable(multi_head_ex multiHead/multi_head_ex.cpp)
target_link_libraries(multi_head_ex
                      ifm3d_camera
                      ifm3d_framegrabber
                      ifm3d_stlimage
                      )
set_target_properties(multi_head_ex
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/examples/cpp/multiHead"
)
