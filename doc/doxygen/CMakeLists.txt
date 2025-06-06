#################################################
# Documentation (Doxygen)
#################################################
find_package(Doxygen)

if (DOXYGEN_FOUND)
    get_directory_property(DOXYGEN_DEFINES COMPILE_DEFINITIONS)
    string(REPLACE ";" " " DOXYGEN_DEFINES "${DOXYGEN_DEFINES}")

    foreach(DOXYGEN_MODULE IN LISTS DOXYGEN_MODULES)
      string(APPEND DOXYGEN_INPUTS "${PROJECT_SOURCE_DIR}/modules/${DOXYGEN_MODULE}/include ")
    endforeach(DOXYGEN_MODULE)

    # set input and output files
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.out)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    add_custom_target(ifm3d_doxygen
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/html/doxygen
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/latex/cpp_api
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
else (DOXYGEN_FOUND)
  message(FATAL_ERROR "Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)