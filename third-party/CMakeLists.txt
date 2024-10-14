set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

add_compile_options($<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-w>$<$<CXX_COMPILER_ID:MSVC>:/w>)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_SHARED_LIBS OFF)
set(BUILD_STATIC_LIBS ON)

if(BUILD_MODULE_PYBIND11)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY "https://github.com/pybind/pybind11"
    GIT_TAG "tags/v2.12.0"
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
  )

  FetchContent_MakeAvailable(pybind11)
  FetchContent_GetProperties(pybind11 SOURCE_DIR pybind_src_dir)
  FetchContent_GetProperties(pybind11 BINARY_DIR pybind_bin_dir)

  list(APPEND CMAKE_MODULE_PATH "${pybind_bin_dir}")
endif(BUILD_MODULE_PYBIND11)
  
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
    GIT_TAG asio-1-30-2
)
FetchContent_MakeAvailable(asio)
add_library(asio INTERFACE)
target_compile_definitions(asio INTERFACE -DASIO_STANDALONE -DASIO_DISABLE_VISIBILITY)
#used for tracing the asio async calls
#target_compile_definitions(asio INTERFACE -DASIO_ENABLE_HANDLER_TRACKING)
target_include_directories(asio SYSTEM INTERFACE
    $<BUILD_INTERFACE:${asio_SOURCE_DIR}/asio/include>
)

#adding CLI11
if(BUILD_MODULE_TOOLS)
  FetchContent_Declare(
    CLI11
    QUIET
    GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
    GIT_TAG v2.4.1
  )
  set(CLI11_BUILD_TESTS OFF CACHE INTERNAL "")
  set(CLI11_EXAMPLES OFF CACHE INTERNAL "")

  FetchContent_MakeAvailable(CLI11)

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

FetchContent_Declare(
  tinyxml2
  FETCHCONTENT_FULLY_DISCONNECTED OFF
  GIT_REPOSITORY "https://github.com/leethomason/tinyxml2.git"
  GIT_TAG "v10.0.0"
  GIT_PROGRESS ON
  GIT_SHALLOW ON
)

if(NOT tinyxml2_POPULATED)
  FetchContent_Populate(tinyxml2)
endif()

add_library(tinyxml2 INTERFACE)
target_sources(tinyxml2 INTERFACE ${tinyxml2_SOURCE_DIR}/tinyxml2.cpp)
target_include_directories(tinyxml2 INTERFACE $<BUILD_INTERFACE:${tinyxml2_SOURCE_DIR}>)

FetchContent_Declare(
  cpp-httplib
    GIT_REPOSITORY "https://github.com/yhirose/cpp-httplib.git"
    GIT_TAG "v0.18.0"
    GIT_PROGRESS ON
    GIT_SHALLOW ON
    PATCH_COMMAND git checkout -q . && git apply --whitespace=fix ${CMAKE_CURRENT_SOURCE_DIR}/patches/cpp-httplib.patch
)
FetchContent_MakeAvailable(cpp-httplib)

unset(BUILD_STATIC_LIBS)
unset(BUILD_SHARED_LIBS)
unset(CMAKE_POSITION_INDEPENDENT_CODE)