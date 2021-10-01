add_subdirectory(doxygen)
add_subdirectory(sphinx)

add_custom_target(docs DEPENDS ifm3d_doxygen ifm3d_sphinx)