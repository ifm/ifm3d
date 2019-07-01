project(IFM3D_SWUPDATER CXX)
set(IFM3D_SWUPDATER_MODULE_NAME "libifm3d-swupdater")

file(COPY ${IFM3D_SWUPDATER_SOURCE_DIR}/include
  DESTINATION ${IFM3D_SWUPDATER_BINARY_DIR}
  )

################################################
## Process child CMakeLists.txt files
################################################
add_subdirectory(src)

if(BUILD_TESTS)
  add_subdirectory(test)
else()
  message(WARNING "Skipping ${IFM3D_SWUPDATER_MODULE_NAME} unit tests!")
endif()

################################################
## Manage installation process
################################################
if(BUILD_SDK_PKG)
  install(DIRECTORY ${IFM3D_SWUPDATER_BINARY_DIR}/include
    DESTINATION ${CPACK_INSTALL_PREFIX}
    COMPONENT swupdater
    PATTERN "*.in" EXCLUDE
    PATTERN ".git" EXCLUDE
    PATTERN "*~" EXCLUDE
   )
endif()
