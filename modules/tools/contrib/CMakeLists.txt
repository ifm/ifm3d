if(NOT CMAKE_CROSSCOMPILING)
  if (UNIX)
    file(COPY ${IFM3D_TOOLS_SOURCE_DIR}/contrib/ifm3d_completions.in
      DESTINATION ${IFM3D_TOOLS_BINARY_DIR}
      )
    configure_file(
      ${IFM3D_TOOLS_BINARY_DIR}/ifm3d_completions.in
      ${IFM3D_TOOLS_BINARY_DIR}/ifm3d_completions
      @ONLY
      )

    install(
      FILES ${IFM3D_TOOLS_BINARY_DIR}/ifm3d_completions
      DESTINATION /etc/bash_completion.d
      COMPONENT tools
      )
  endif()
endif()