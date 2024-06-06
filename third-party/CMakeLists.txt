set(CMAKE_CXX_STANDARD 14)

include(FetchContent)

add_compile_options($<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:-w>$<$<CXX_COMPILER_ID:MSVC>:/w>)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_SHARED_LIBS OFF)
set(BUILD_STATIC_LIBS ON)

#### CURL ####
FetchContent_Declare(
  curl
  FETCHCONTENT_FULLY_DISCONNECTED ON
  GIT_REPOSITORY "https://github.com/curl/curl.git"
  GIT_TAG "curl-7_78_0"
  GIT_PROGRESS TRUE
  GIT_SHALLOW TRUE
)

FetchContent_GetProperties(curl)
if(NOT curl_POPULATED)
  FetchContent_Populate(curl )
endif()

set(HTTP_ONLY ON)
set(CURL_ENABLE_SSL OFF)
add_subdirectory(${curl_SOURCE_DIR} ${curl_BINARY_DIR} EXCLUDE_FROM_ALL)
unset(HTTP_ONLY)
unset(CURL_ENABLE_SSL)


#### XMLRPC-C ####
FetchContent_Declare(
  xmlrpc-c
  FETCHCONTENT_FULLY_DISCONNECTED OFF
  GIT_REPOSITORY "https://github.com/ifm/xmlrpc-c.git"
  GIT_TAG "31b26e833bf00265a500ae0afe58775703670654"
  GIT_PROGRESS ON
  GIT_SHALLOW ON
  PATCH_COMMAND git apply  --ignore-space-change --ignore-whitespace ${CMAKE_CURRENT_SOURCE_DIR}/patches/xmlrpc.patch
)

if(NOT xmlrpc-c_POPULATED)
  FetchContent_Populate(xmlrpc-c)
endif()

set(BUILD_TOOLS OFF)
set(ENABLE_EXAMPLES OFF)
add_subdirectory(${xmlrpc-c_SOURCE_DIR} ${xmlrpc-c_BINARY_DIR} EXCLUDE_FROM_ALL)
unset(BUILD_TOOLS)
unset(ENABLE_EXAMPLES)

target_include_directories(xmlrpc PUBLIC
  $<BUILD_INTERFACE:${xmlrpc-c_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${xmlrpc-c_SOURCE_DIR}/lib/util/include>
  $<BUILD_INTERFACE:${xmlrpc-c_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${xmlrpc-c_BINARY_DIR}>
)

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

unset(BUILD_STATIC_LIBS)
unset(BUILD_SHARED_LIBS)
unset(CMAKE_POSITION_INDEPENDENT_CODE)