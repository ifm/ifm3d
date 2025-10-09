ifm3d_module(tools
  LINK_LIBRARIES 
    CLI11::CLI11
    $<BUILD_INTERFACE:fmt::fmt-header-only>
    $<BUILD_INTERFACE:tinyxml2>
    $<BUILD_INTERFACE:httplib::httplib>
  REQUIRED_MODULES
    common
    device
)

if (BUILD_MODULE_SWUPDATER)
  ifm3d_link_modules(ifm3d_tools MODULES swupdater)
else()
  ifm3d_disable_sources(tools GLOB_RECURSE src/common/swupdater/*.cpp)
endif()

if (BUILD_MODULE_FRAMEGRABBER)
  ifm3d_link_modules(ifm3d_tools MODULES framegrabber)
else()
  ifm3d_disable_sources(tools GLOB_RECURSE src/common/fg/*.cpp)
endif()

add_executable(ifm3d ifm3d.cpp)
target_link_libraries(ifm3d ifm3d_tools)
target_compile_definitions(ifm3d PRIVATE IFM3D_STATIC_LIB)
install(TARGETS ifm3d
  DESTINATION ${_bin} COMPONENT ifm3d
  COMPONENT ifm3d
)


# if (BUILD_MODULE_FRAMEGRABBER)
#   target_link_libraries(ifm3d_tools INTERFACE ifm3d_framegrabber)
# endif()

# if (BUILD_MODULE_SWUPDATER)
#   target_link_libraries(ifm3d_tools INTERFACE ifm3d_swupdater)
# endif()



# project(IFM3D_TOOLS CXX)
# set(IFM3D_TOOLS_MODULE_NAME "libifm3d-tools")


# ################################################
# ## Process child CMakeLists.txt files
# ################################################
# add_subdirectory(src)
# add_subdirectory(contrib)

# ################################################
# ## Manage installation process
# ################################################
# if(BUILD_SDK_PKG)
#   if (NOT BUILD_MODULE_FRAMEGRABBER)
#     list(APPEND PKG_EXCLUDES PATTERN "include/ifm3d/tools/fg" EXCLUDE)
#   endif()
#   if (NOT BUILD_MODULE_SWUPDATER)
#     list(APPEND PKG_EXCLUDES PATTERN "include/ifm3d/tools/swupdater" EXCLUDE)
#   endif()

#   install(DIRECTORY ${IFM3D_TOOLS_SOURCE_DIR}/include/
#     DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#     COMPONENT ifm3d
#     PATTERN "*.in" EXCLUDE
#     PATTERN ".git" EXCLUDE
#     PATTERN "*~" EXCLUDE
#     ${PKG_EXCLUDES}
#     )
# endif()
