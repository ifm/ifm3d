find_package(ifm3d 0.91.0 CONFIG
  REQUIRED COMPONENTS camera
  )

add_executable(configuration_ex configuration_ex.cpp)
target_link_libraries(configuration_ex
                      ifm3d::camera
                      )