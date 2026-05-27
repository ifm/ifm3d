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
target_compile_options(ifm3d PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

install(TARGETS ifm3d
  DESTINATION ${_bin} COMPONENT ifm3d
  COMPONENT ifm3d
)

#################################################
# Shell completion installation (Linux only)
#################################################
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND NOT CMAKE_CROSSCOMPILING)
  # Directory to store generated completion scripts
  set(IFM3D_COMPLETION_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/completions")

  # Generate wrapper scripts at configure time
  file(GENERATE
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/gen_bash_completion.sh
    CONTENT "#!/bin/sh\nmkdir -p '${IFM3D_COMPLETION_OUTPUT_DIR}'\n'$<TARGET_FILE:ifm3d>' generate-completion bash > '${IFM3D_COMPLETION_OUTPUT_DIR}/ifm3d'\n"
  )
  file(GENERATE
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/gen_zsh_completion.sh
    CONTENT "#!/bin/sh\nmkdir -p '${IFM3D_COMPLETION_OUTPUT_DIR}'\n'$<TARGET_FILE:ifm3d>' generate-completion zsh > '${IFM3D_COMPLETION_OUTPUT_DIR}/_ifm3d'\n"
  )

  # Generate bash completion script after building ifm3d
  add_custom_command(
    OUTPUT ${IFM3D_COMPLETION_OUTPUT_DIR}/ifm3d
    COMMAND sh ${CMAKE_CURRENT_BINARY_DIR}/gen_bash_completion.sh
    DEPENDS ifm3d
    COMMENT "Generating bash completion script for ifm3d"
  )

  # Generate zsh completion script after building ifm3d
  add_custom_command(
    OUTPUT ${IFM3D_COMPLETION_OUTPUT_DIR}/_ifm3d
    COMMAND sh ${CMAKE_CURRENT_BINARY_DIR}/gen_zsh_completion.sh
    DEPENDS ifm3d
    COMMENT "Generating zsh completion script for ifm3d"
  )

  add_custom_target(ifm3d_completions ALL
    DEPENDS
      ${IFM3D_COMPLETION_OUTPUT_DIR}/ifm3d
      ${IFM3D_COMPLETION_OUTPUT_DIR}/_ifm3d
  )

  # Install bash completion to standard directory
  # /usr/share/bash-completion/completions/ is the modern standard for lazy-loading
  install(
    FILES ${IFM3D_COMPLETION_OUTPUT_DIR}/ifm3d
    DESTINATION share/bash-completion/completions
    COMPONENT ifm3d
  )

  # Install zsh completion to standard directory
  # /usr/share/zsh/vendor-completions/ is used by Debian/Ubuntu
  install(
    FILES ${IFM3D_COMPLETION_OUTPUT_DIR}/_ifm3d
    DESTINATION share/zsh/vendor-completions
    COMPONENT ifm3d
  )
endif()

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
