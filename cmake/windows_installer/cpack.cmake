set(CPACK_GENERATOR NSIS)

set(CPACK_PACKAGE_NAME "ifm3d")
set(CPACK_PACKAGE_VENDOR "ifm")
set(CPACK_PACKAGE_CONTACT "support.robotics@ifm.com")
set(CPACK_PACKAGE_VERSION ${ROOT_PROJECT_VERSION})
set(CPACK_PACKAGE_VERSION_MAJOR ${ROOT_PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${ROOT_PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${ROOT_PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_DIRECTORY ${CMAKE_BINARY_DIR})
set(CPACK_PACKAGE_ICON "${CMAKE_SOURCE_DIR}/cmake/windows_installer\\\\ifm_logo.ico")

string(TOLOWER ${CMAKE_SYSTEM_NAME} _sys)
string(TOLOWER ${PROJECT_NAME} _project_lower)
set(CPACK_SOURCE_GENERATOR ZIP)
set(CPACK_PACKAGE_FILE_NAME "${_project_lower}_${_sys}_${PROJECT_VERSION}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${_project_lower}_${PROJECT_VERSION}")

set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.md")

set(CPACK_OUTPUT_FILE_PREFIX "${CMAKE_SOURCE_DIR}/ifm3d_installer")

set(CPACK_NSIS_MODIFY_PATH ON)
set(CPACK_NSIS_DISPLAY_NAME "ifm3d")
set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\ifm3d.exe")
set(CPACK_NSIS_HELP_LINK "https://github.com/ifm")
set(CPACK_NSIS_URL_INFO_ABOUT "http://www.ifm.com/")
set(CPACK_NSIS_CONTACT "support.robotics@ifm.com")
set(CPACK_NSIS_MUI_ICON "${CMAKE_SOURCE_DIR}/cmake/windows_installer\\\\ifm_logo.ico")
set(CPACK_NSIS_MUI_UNIICON "${CMAKE_SOURCE_DIR}/cmake/windows_installer\\\\ifm_logo.ico")

# not .gitignore as its regex syntax is distinct
file(READ ${CMAKE_CURRENT_LIST_DIR}/.cpack_ignore _cpack_ignore)
string(REGEX REPLACE "\n" ";" _cpack_ignore ${_cpack_ignore})
set(CPACK_SOURCE_IGNORE_FILES "${_cpack_ignore}")

install(FILES ${CPACK_RESOURCE_FILE_README} ${CPACK_RESOURCE_FILE_LICENSE}
  DESTINATION share/docs/${PROJECT_NAME})

include(CPack)