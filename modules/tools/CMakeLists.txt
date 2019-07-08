project(IFM3D_TOOLS CXX)
set(IFM3D_TOOLS_MODULE_NAME "libifm3d-tools")

file(COPY ${IFM3D_TOOLS_SOURCE_DIR}/include
  DESTINATION ${IFM3D_TOOLS_BINARY_DIR}
  )
if (NOT BUILD_MODULE_FRAMEGRABBER)
  file(REMOVE_RECURSE ${IFM3D_TOOLS_BINARY_DIR}/include/ifm3d/tools/fg)
endif()
if (NOT BUILD_MODULE_SWUPDATER)
  file(REMOVE_RECURSE ${IFM3D_TOOLS_BINARY_DIR}/include/ifm3d/tools/swupdater)
endif()

################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
add_subdirectory(contrib)

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_TOOLS_BINARY_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    COMPONENT tools
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )
endif()
