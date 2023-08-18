cmake_minimum_required(VERSION 3.11)

project(ifm3d_playground CXX)

#Global compiler flags
set(CMAKE_BUILD_TYPE Release) # Release or Debug
set(CMAKE_CXX_EXTENSIONS OFF) # OFF -> -std=c++14, ON -> -std=gnu++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

#find ifm3d on the system
find_package(ifm3d 1.3.3 CONFIG
  REQUIRED COMPONENTS device framegrabber
  )
## Configuration example
add_executable(ifm3d_playground ifm3d_playground.cpp)
target_link_libraries(ifm3d_playground
                      ifm3d::device
                      ifm3d::framegrabber
                      )