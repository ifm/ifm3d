project(IFM3D_TOOLS CXX)
set(IFM3D_TOOLS_MODULE_NAME "libifm3d-tools")

file(COPY ${IFM3D_TOOLS_SOURCE_DIR}/include
  DESTINATION ${IFM3D_TOOLS_BINARY_DIR}
  )

################################################
## Bring in dependent projects
################################################
find_package(Boost REQUIRED COMPONENTS program_options)

################################################
## Compiler and linker settings
################################################
include_directories(
  ${IFM3D_TOOLS_BINARY_DIR}/include
  ${IFM3D_CAMERA_BINARY_DIR}/include
  ${Boost_INCLUDE_DIRS}
  )

################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_TOOLS_BINARY_DIR}/include
   DESTINATION ${CPACK_INSTALL_PREFIX}
   PATTERN "*.in" EXCLUDE
   PATTERN ".git" EXCLUDE
   PATTERN "*~" EXCLUDE
   )
endif()

################################################
## Manage packaging process
################################################

################################################
## Manage deployment process
################################################
