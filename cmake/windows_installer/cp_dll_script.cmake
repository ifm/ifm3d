# Find third party DLLS from _deps
file(GLOB_RECURSE deps_DLLs
  "_deps/*.dll"
)

# Copy third party DLLs to CMAKE_INSTALL_PREFIX bin directory
file(COPY ${deps_DLLs} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/)
