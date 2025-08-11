ifm3d_module(device
  LINK_LIBRARIES 
    tinyxml2
    $<BUILD_INTERFACE:asio>
    $<BUILD_INTERFACE:fmt::fmt-header-only>
    $<BUILD_INTERFACE:httplib::httplib>
  REQUIRED_MODULES
    common
)

if(BUILD_MODULE_CRYPTO)
  target_link_libraries(ifm3d_device INTERFACE ifm3d_crypto)
endif()


# project(IFM3D_DEVICE CXX)
# set(IFM3D_DEVICE_MODULE_NAME "libifm3d-device")

configure_file(
  ${IFM3D_DEVICE_SOURCE_DIR}/include/ifm3d/device/version.h.in
  ${IFM3D_DEVICE_BINARY_DIR}/include/ifm3d/device/version.h
  @ONLY
)


# include_directories(${IFM3D_DEVICE_SOURCE_DIR}/include ${IFM3D_DEVICE_BINARY_DIR}/include)


# ################################################
# ## Process child CMakeLists.txt files
# ################################################
# add_subdirectory(src)
# if(BUILD_TESTS)
#   add_subdirectory(test)
# else()
#   message(WARNING "Skipping ${IFM3D_DEVICE_MODULE_NAME} unit tests!")
# endif()

# ################################################
# ## Manage installation process
# ################################################
# if(BUILD_SDK_PKG)
#   install(DIRECTORY ${IFM3D_DEVICE_SOURCE_DIR}/include/ ${IFM3D_DEVICE_BINARY_DIR}/include/
#     DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
#     COMPONENT ifm3d
#     PATTERN "*.in" EXCLUDE
#     PATTERN ".git" EXCLUDE
#     PATTERN "*~" EXCLUDE
#   )
# endif()
