project(IFM3D_FG2 CXX)
set(IFM3D_FG_MODULE_NAME "libifm3d-framegrabber")

include_directories(${IFM3D_FG_SOURCE_DIR}/include)

################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_FG_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_FG_SOURCE_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    COMPONENT framegrabber
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
   )
endif()
