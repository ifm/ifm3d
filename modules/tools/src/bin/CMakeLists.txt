add_executable(ifm3d ifm3d.cpp)
target_link_libraries(ifm3d ifm3d_tools)
target_compile_definitions(ifm3d PRIVATE IFM3D_STATIC_LIB)
install(TARGETS ifm3d
  DESTINATION ${_bin} COMPONENT ifm3d
  COMPONENT ifm3d
)
