find_package(ifm3d 0.91.0 CONFIG
  REQUIRED COMPONENTS camera framegrabber image
  )

add_executable(multi_head_ex multi_head_ex.cpp)
target_link_libraries(multi_head_ex
                      ifm3d::camera
                      ifm3d::framegrabber
                      ifm3d::image
                      )