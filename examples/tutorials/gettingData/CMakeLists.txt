find_package(ifm3d 0.91.0 CONFIG
  REQUIRED COMPONENTS camera framegrabber image
  )

add_executable(getting_data_ex getting_data_ex.cpp)
target_link_libraries(getting_data_ex
                      ifm3d::camera
                      ifm3d::framegrabber
                      ifm3d::image
                      )