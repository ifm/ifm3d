find_package(Boost REQUIRED COMPONENTS system regex date_time)

project(IFM3D_PCICCLIENT CXX)
set(IFM3D_PCICCLIENT_MODULE_NAME "libifm3d-pcicclient")

file(COPY ${IFM3D_PCICCLIENT_SOURCE_DIR}/include
  DESTINATION ${IFM3D_PCICCLIENT_BINARY_DIR}
  )

################################################
## Compiler and linker settings
################################################
include_directories(
  ${IFM3D_PCICCLIENT_BINARY_DIR}/include
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
  message(WARNING "Skipping ${IFM3D_PCICCLIENT_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_PCICCLIENT_BINARY_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    COMPONENT pcicclient
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
    )
endif()
