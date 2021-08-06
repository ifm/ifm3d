##############################################################
## finding the ifm3d lib.
##############################################################
find_package(ifm3d 0.91.0 CONFIG
  REQUIRED COMPONENTS camera framegrabber image
  )

add_executable(basic_usage_ex basic_usage_ex.cpp)
target_link_libraries(basic_usage_ex
                      ifm3d::camera
                      ifm3d::framegrabber
                      ifm3d::image
                      )

add_subdirectory(tutorials)
