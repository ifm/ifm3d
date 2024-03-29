set(CMAKE_CXX_STANDARD 14)

include(FetchContent)
if(BUILD_IN_DEPS)
  list(APPEND BUILTIN_INCLUDES "" PARENT_SCOPE)

  add_compile_options($<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-w>$<$<CXX_COMPILER_ID:MSVC>:/w>)

  FetchContent_Declare(
    curl
    FETCHCONTENT_FULLY_DISCONNECTED ON
    GIT_REPOSITORY "https://github.com/curl/curl.git"
    GIT_TAG "curl-7_78_0"
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
  )

  FetchContent_MakeAvailable(curl)
  FetchContent_GetProperties(curl)
  if(NOT curl_POPULATED)
    FetchContent_Populate(curl )
  endif()
  FetchContent_GetProperties(curl SOURCE_DIR src_dir)
  FetchContent_GetProperties(curl BINARY_DIR bin_dir)
  list( APPEND BUILTIN_INCLUDES ${src_dir}/include  ${bin_dir}/include/curl)
  # make curl includes available to all projects
  include_directories( ${BUILTIN_INCLUDES})
  list(APPEND CMAKE_MODULE_PATH "${bin_dir}")

  FetchContent_Declare(
    xmlrpc-c
    FETCHCONTENT_FULLY_DISCONNECTED OFF
    GIT_REPOSITORY "https://github.com/ifm/xmlrpc-c.git"
    GIT_TAG "31b26e833bf00265a500ae0afe58775703670654"
    GIT_PROGRESS ON
    GIT_SHALLOW ON
    PATCH_COMMAND git apply  --ignore-space-change --ignore-whitespace ${CMAKE_CURRENT_SOURCE_DIR}/patches/xmlrpc.patch
  )

  #xmlrpc configure options
  set(BUILD_TOOLS OFF)
  set(ENABLE_EXAMPLES OFF)
  FetchContent_Populate(xmlrpc-c )
  FetchContent_GetProperties(xmlrpc-c SOURCE_DIR xmlrpc_src_dir)
  FetchContent_GetProperties(xmlrpc-c BINARY_DIR xmlrpc_bin_dir)

  list(APPEND CMAKE_MODULE_PATH "${xmlrpc_bin_dir}")
  add_subdirectory(${xmlrpc_src_dir} ${xmlrpc_bin_dir})
  unset(BUILD_TOOLS)
  unset(ENABLE_EXAMPLES)

  list(APPEND BUILTIN_INCLUDES ${xmlrpc_src_dir}/include ${xmlrpc_src_dir}/lib/util/include ${xmlrpc_bin_dir}/include  ${xmlrpc_bin_dir})
  set( BUILTIN_INCLUDES ${BUILTIN_INCLUDES} PARENT_SCOPE)
  if(WIN32)
    target_compile_definitions(xmlrpc PUBLIC _HAS_AUTO_PTR_ETC=1)
  endif()

  if(BUILD_MODULE_PYBIND11)
    FetchContent_Declare(
     pybind11
      GIT_REPOSITORY "https://github.com/pybind/pybind11"
      GIT_TAG "tags/v2.10.1"
      GIT_SHALLOW TRUE
      GIT_PROGRESS TRUE
    )

    FetchContent_MakeAvailable(pybind11)
    FetchContent_GetProperties(pybind11 SOURCE_DIR pybind_src_dir)
    FetchContent_GetProperties(pybind11 BINARY_DIR pybind_bin_dir)

    list(APPEND CMAKE_MODULE_PATH "${pybind_bin_dir}")
    list(APPEND BUILTIN_INCLUDES ${pybind_src_dir}/include ${pybind_bin_dir}/include  ${pybind_bin_dir})
  endif(BUILD_MODULE_PYBIND11)

  set( BUILTIN_INCLUDES ${BUILTIN_INCLUDES} PARENT_SCOPE)
endif(BUILD_IN_DEPS)

FetchContent_Declare(
  fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt.git
    GIT_TAG 8.0.1
)
FetchContent_MakeAvailable(fmt)

#adding asio
FetchContent_Declare(
  asio
    GIT_REPOSITORY https://github.com/chriskohlhoff/asio
    GIT_TAG asio-1-26-0
)
FetchContent_MakeAvailable(asio)
add_library(asio INTERFACE)
target_compile_definitions(asio INTERFACE -DASIO_STANDALONE -DASIO_DISABLE_VISIBILITY)
#used for tracing the asio async calls
#target_compile_definitions(asio INTERFACE -DASIO_ENABLE_HANDLER_TRACKING)
target_include_directories(asio SYSTEM INTERFACE
    $<BUILD_INTERFACE:${asio_SOURCE_DIR}/asio/include>
)

#adding cxxopts
if(BUILD_MODULE_TOOLS)
  FetchContent_Declare(
    cxxopts
      GIT_REPOSITORY https://github.com/jarro2783/cxxopts
      GIT_TAG v3.1.1
      GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(cxxopts)
endif(BUILD_MODULE_TOOLS)

#adding websocketpp
if(BUILD_MODULE_SWUPDATER)
  FetchContent_Declare(
    websocketpp
      GIT_REPOSITORY https://github.com/zaphoyd/websocketpp.git
      GIT_TAG 0.8.2
  )
  FetchContent_Populate(websocketpp)
  FetchContent_GetProperties(websocketpp SOURCE_DIR websocketpp_src_dir)

  add_library(websocketpp_wrapper INTERFACE)

  target_include_directories(websocketpp_wrapper INTERFACE
        $<BUILD_INTERFACE:${websocketpp_src_dir}>
  )
  target_link_libraries(websocketpp_wrapper INTERFACE asio)

  if(MSVC)
    target_compile_definitions (websocketpp_wrapper INTERFACE -D_WEBSOCKETPP_CPP11_FUNCTIONAL_)
    target_compile_definitions (websocketpp_wrapper INTERFACE -D_WEBSOCKETPP_CPP11_SYSTEM_ERROR_)
    target_compile_definitions (websocketpp_wrapper INTERFACE -D_WEBSOCKETPP_CPP11_RANDOM_DEVICE_)
    target_compile_definitions (websocketpp_wrapper INTERFACE -D_WEBSOCKETPP_CPP11_MEMORY_)
    target_compile_definitions (websocketpp_wrapper INTERFACE -D_WEBSOCKETPP_CPP11_STRICT_)

  else()
    target_compile_definitions (websocketpp_wrapper INTERFACE -D_WEBSOCKETPP_CPP11_STL_)
  endif()
    target_compile_definitions(websocketpp_wrapper INTERFACE ASIO_DISABLE_STD_EXPERIMENTAL_STRING_VIEW)

endif(BUILD_MODULE_SWUPDATER)