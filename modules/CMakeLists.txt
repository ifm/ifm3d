add_subdirectory(common)
list(APPEND DOXYGEN_MODULES "common")

if (BUILD_MODULE_CRYPTO)
  add_subdirectory(crypto)
  list(APPEND DOXYGEN_MODULES "crypto")
endif()

add_subdirectory(device)
list(APPEND DOXYGEN_MODULES "device")

if(BUILD_MODULE_FRAMEGRABBER)
  add_subdirectory(framegrabber)
  list(APPEND DOXYGEN_MODULES "framegrabber")
endif()

if(BUILD_MODULE_SWUPDATER)
  add_subdirectory(swupdater)
  list(APPEND DOXYGEN_MODULES "swupdater")
endif()

if(BUILD_MODULE_PCICCLIENT)
  add_subdirectory(pcicclient)
  list(APPEND DOXYGEN_MODULES "pcicclient")
endif()

if(BUILD_MODULE_PYBIND11)
  add_subdirectory(pybind11)
endif()

if(BUILD_MODULE_DESERIALIZE)
  add_subdirectory(deserialize)
  list(APPEND DOXYGEN_MODULES "deserialize")
endif()

if (BUILD_SHARED_LIBS)
  add_subdirectory(libifm3d)
endif()

if(BUILD_MODULE_TOOLS)
  add_subdirectory(tools)
endif()

set(DOXYGEN_MODULES "${DOXYGEN_MODULES}" PARENT_SCOPE)