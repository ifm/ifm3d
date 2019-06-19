project(IFM3D_PYBIND11 CXX)
set(IFM3D_PYBIND11_MODULE_NAME "libifm3d-pybind11")

if(FORCE_OPENCV3)
  find_package(OpenCV 3 REQUIRED COMPONENTS core)
elseif(FORCE_OPENCV2)
  find_package(OpenCV 2.4 REQUIRED COMPONENTS core)
else()
  find_package(OpenCV REQUIRED COMPONENTS core)
endif()

# needs to be done before the target is declared
link_directories(${OpenCV_LIBRARY_DIRS})


################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_PYBIND11_MODULE_NAME} unit tests!")
endif()
