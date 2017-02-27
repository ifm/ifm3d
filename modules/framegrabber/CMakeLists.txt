project(IFM3D_FG CXX)
set(IFM3D_FG_MODULE_NAME "libifm3d-framegrabber")

file(COPY ${IFM3D_FG_SOURCE_DIR}/include
  DESTINATION ${IFM3D_FG_BINARY_DIR}
  )

################################################
## Bring in dependent projects
################################################
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Compiler and linker settings
################################################
include_directories(
  ${IFM3D_FG_BINARY_DIR}/include
  ${IFM3D_CAMERA_BINARY_DIR}/include
  ${Boost_INCLUDE_DIRS}
  )

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
  install(DIRECTORY ${IFM3D_FG_BINARY_DIR}/include
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
