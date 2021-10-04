project(IFM3D_PYBIND11 CXX)
set(IFM3D_PYBIND11_MODULE_NAME "libifm3d-pybind11")

################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_PYBIND11_MODULE_NAME} unit tests!")
endif()
