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
# Shell completion installation
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

elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows" AND NOT CMAKE_CROSSCOMPILING)
  # -------------------------------------------------------------------
  # PowerShell tab completion for Windows
  # -------------------------------------------------------------------
  # Strategy (mirrors the Linux bash/zsh lazy-load approach):
  #   1. After the ifm3d binary is built, run it via a generated wrapper
  #      script to produce the completion .ps1 file.
  #   2. Install the .ps1 to share/ifm3d/completions/ifm3d.ps1.
  #   3. Ship register_pwsh_completion.ps1 / unregister_pwsh_completion.ps1
  #      alongside so the NSIS installer (and manual users) can wire the
  #      script into the PowerShell profile automatically.
  #
  # NSIS installer: cpack.cmake runs register_pwsh_completion.ps1 -AllUsers
  #   which writes a guarded block to the AllUsersAllHosts profile so that
  #   completion is available to every user on the machine immediately after
  #   installation - no per-user action required.
  #
  # Build-from-source: run register_pwsh_completion.ps1 once as instructed
  #   by the post-install message printed below.
  # -------------------------------------------------------------------

  set(IFM3D_COMPLETION_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/completions")

  # file(GENERATE) supports $<TARGET_FILE:> and $<CONFIG> generator expressions.
  # One wrapper script is generated per build configuration so that MSVC
  # multi-config generators (Debug / Release) each reference the correct binary.
  # The output file (ifm3d.ps1) is config-independent: completion content is
  # identical regardless of build type.
  file(GENERATE
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/gen_pwsh_completion_$<CONFIG>.ps1"
    CONTENT "New-Item -ItemType Directory -Force -Path '${IFM3D_COMPLETION_OUTPUT_DIR}' | Out-Null\n& '$<TARGET_FILE:ifm3d>' generate-completion pwsh | Set-Content -Encoding UTF8 '${IFM3D_COMPLETION_OUTPUT_DIR}/ifm3d.ps1'\n"
  )

  # Generate the completion script at build time, after ifm3d is built.
  add_custom_command(
    OUTPUT "${IFM3D_COMPLETION_OUTPUT_DIR}/ifm3d.ps1"
    COMMAND powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass
            -File "${CMAKE_CURRENT_BINARY_DIR}/gen_pwsh_completion_$<CONFIG>.ps1"
    DEPENDS ifm3d
    COMMENT "Generating PowerShell completion script for ifm3d"
  )

  add_custom_target(ifm3d_completions ALL
    DEPENDS "${IFM3D_COMPLETION_OUTPUT_DIR}/ifm3d.ps1"
  )

  # Install the generated completion script
  install(
    FILES "${IFM3D_COMPLETION_OUTPUT_DIR}/ifm3d.ps1"
    DESTINATION share/ifm3d/completions
    COMPONENT ifm3d
  )

  # Install the profile-management helper scripts (used by NSIS and manual installs)
  install(
    FILES
      "${CMAKE_SOURCE_DIR}/cmake/windows_installer/register_pwsh_completion.ps1"
      "${CMAKE_SOURCE_DIR}/cmake/windows_installer/unregister_pwsh_completion.ps1"
    DESTINATION share/ifm3d/completions
    COMPONENT ifm3d
  )

  # For build-from-source installs (not the NSIS installer), print instructions.
  # Use bracket notation so ${CMAKE_INSTALL_PREFIX} is deferred to install-time.
  if(NOT CREATE_WIN_INSTALLER)
    install(CODE [[
      message(STATUS "")
      message(STATUS "ifm3d: PowerShell tab completion installed.")
      message(STATUS "  Script: ${CMAKE_INSTALL_PREFIX}/share/ifm3d/completions/ifm3d.ps1")
      message(STATUS "")
      message(STATUS "  To REGISTER (run once in PowerShell):")
      message(STATUS "    powershell -ExecutionPolicy Bypass -File \"${CMAKE_INSTALL_PREFIX}/share/ifm3d/completions/register_pwsh_completion.ps1\" -InstallDir \"${CMAKE_INSTALL_PREFIX}\"")
      message(STATUS "  Then restart PowerShell (or run '. \$PROFILE') to activate.")
      message(STATUS "")
      message(STATUS "  To UNREGISTER (removes tab completion from your profile):")
      message(STATUS "    powershell -ExecutionPolicy Bypass -File \"${CMAKE_INSTALL_PREFIX}/share/ifm3d/completions/unregister_pwsh_completion.ps1\"")
      message(STATUS "  Or, to unregister for all users (requires elevation):")
      message(STATUS "    powershell -ExecutionPolicy Bypass -File \"${CMAKE_INSTALL_PREFIX}/share/ifm3d/completions/unregister_pwsh_completion.ps1\" -AllUsers")
      message(STATUS "")
      message(STATUS "  To uninstall files and unregister in one step:")
      message(STATUS "    cmake --build <build-dir> --target uninstall")
      message(STATUS "")
    ]] COMPONENT ifm3d)
  endif()

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
