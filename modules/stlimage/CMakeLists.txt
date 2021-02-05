project(IFM3D_STLIMAGE CXX)
set(IFM3D_STLIMAGE_MODULE_NAME "libifm3d-stlimage")

################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
if(BUILD_TESTS)
#  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_STLIMAGE_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    COMPONENT image
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )
endif()
