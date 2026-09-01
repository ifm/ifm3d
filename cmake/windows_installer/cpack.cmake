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

# ---------------------------------------------------------------------------
# PowerShell tab completion: automatic profile registration
#
# After installation the completion script lives at:
#   $INSTDIR\share\ifm3d\completions\ifm3d.ps1
#
# The register / unregister PowerShell scripts are installed alongside it and
# write / remove a guarded block in the machine-wide PowerShell profile
# (AllUsersAllHosts) so that tab completion is available to every user
# immediately, without any per-user action.
#
# CMake bracket strings [=[ ... ]=] are used so that $INSTDIR and $WINDIR
# are NOT expanded by CMake but remain as NSIS variables to be resolved at
# package-install / package-uninstall time.
#
# NSIS string notes:
#   - Double-quoted NSIS strings expand $VARIABLE and require $\" for a
#     literal double-quote character inside the string.
#   - nsExec::ExecToLog runs hidden and returns an exit code that we must
#     check explicitly so install/uninstall does not silently skip failures.
# ---------------------------------------------------------------------------
set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS [=[
  IfFileExists \"$PROGRAMFILES64/PowerShell/7/pwsh.exe\" 0 use_legacy_pwsh
  nsExec::ExecToLog '\"$PROGRAMFILES64/PowerShell/7/pwsh.exe\" -NoLogo -NoProfile -NonInteractive -ExecutionPolicy Bypass -WindowStyle Hidden -File \"$INSTDIR/share/ifm3d/completions/register_pwsh_completion.ps1\" -InstallDir \"$INSTDIR\" -AllUsers'
  Pop $0
  Goto check_pwsh_registration
use_legacy_pwsh:
  nsExec::ExecToLog '\"$WINDIR/System32/WindowsPowerShell/v1.0/powershell.exe\" -NoLogo -NoProfile -NonInteractive -ExecutionPolicy Bypass -WindowStyle Hidden -File \"$INSTDIR/share/ifm3d/completions/register_pwsh_completion.ps1\" -InstallDir \"$INSTDIR\" -AllUsers'
  Pop $0
check_pwsh_registration:
  StrCmp $0 0 pwsh_registration_done
  MessageBox MB_OK \"Warning: ifm3d PowerShell tab completion registration failed (exit code $0).\"
pwsh_registration_done:
]=])

set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS [=[
  IfFileExists \"$PROGRAMFILES64/PowerShell/7/pwsh.exe\" 0 use_legacy_unpwsh
  nsExec::ExecToLog '\"$PROGRAMFILES64/PowerShell/7/pwsh.exe\" -NoLogo -NoProfile -NonInteractive -ExecutionPolicy Bypass -WindowStyle Hidden -File \"$INSTDIR/share/ifm3d/completions/unregister_pwsh_completion.ps1\" -AllUsers'
  Pop $0
  Goto check_pwsh_unregistration
use_legacy_unpwsh:
  nsExec::ExecToLog '\"$WINDIR/System32/WindowsPowerShell/v1.0/powershell.exe\" -NoLogo -NoProfile -NonInteractive -ExecutionPolicy Bypass -WindowStyle Hidden -File \"$INSTDIR/share/ifm3d/completions/unregister_pwsh_completion.ps1\" -AllUsers'
  Pop $0
check_pwsh_unregistration:
  StrCmp $0 0 pwsh_unregistration_done
  MessageBox MB_OK \"Warning: ifm3d PowerShell tab completion removal failed (exit code $0).\"
pwsh_unregistration_done:
]=])

# not .gitignore as its regex syntax is distinct
file(READ ${CMAKE_CURRENT_LIST_DIR}/.cpack_ignore _cpack_ignore)
string(REGEX REPLACE "\n" ";" _cpack_ignore ${_cpack_ignore})
set(CPACK_SOURCE_IGNORE_FILES "${_cpack_ignore}")

install(FILES ${CPACK_RESOURCE_FILE_README} ${CPACK_RESOURCE_FILE_LICENSE}
  DESTINATION share/docs/${PROJECT_NAME})

include(CPack)