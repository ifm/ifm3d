project(IFM3D_TOOLS CXX)
set(IFM3D_TOOLS_MODULE_NAME "libifm3d-tools")


################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)
add_subdirectory(contrib)

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  if (NOT BUILD_MODULE_FRAMEGRABBER)
    list(APPEND PKG_EXCLUDES PATTERN "include/ifm3d/tools/fg" EXCLUDE)
  endif()
  if (NOT BUILD_MODULE_SWUPDATER)
    list(APPEND PKG_EXCLUDES PATTERN "include/ifm3d/tools/swupdater" EXCLUDE)
  endif()

  install(DIRECTORY ${IFM3D_TOOLS_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    COMPONENT ifm3d
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    ${PKG_EXCLUDES}
    )
endif()
