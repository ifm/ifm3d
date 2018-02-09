add_executable(ifm3d ifm3d.cpp)
target_link_libraries(ifm3d
  ifm3d_tools_shared
  )
install(TARGETS ifm3d
  DESTINATION ${_bin} COMPONENT tools
  )
