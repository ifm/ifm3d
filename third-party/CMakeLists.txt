
if(BUILD_IN_DEPS)
cmake_minimum_required(VERSION 3.11)

include(FetchContent)
list(APPEND BUILTIN_INCLUDES "" PARENT_SCOPE)

FetchContent_Declare(
  curl
  FETCHCONTENT_FULLY_DISCONNECTED ON
  GIT_REPOSITORY "https://github.com/curl/curl.git"
  GIT_TAG "curl-7_78_0"
  GIT_PROGRESS TRUE
  GIT_SHALLOW TRUE
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/third-party/downloads/curl"
)

FetchContent_MakeAvailable(curl)
FetchContent_GetProperties(curl)
if(NOT curl_POPULATED)
  FetchContent_Populate(curl )
endif()
FetchContent_GetProperties(curl SOURCE_DIR src_dir)
FetchContent_GetProperties(curl BINARY_DIR bin_dir)
list( APPEND BUILTIN_INCLUDES ${src_dir}/include  ${bin_dir}/include/curl)
# make curl includes available to complete projects
include_directories( ${BUILTIN_INCLUDES})
list(APPEND CMAKE_MODULE_PATH "${bin_dir}")

FetchContent_Declare(
  xmlrpc-c
  FETCHCONTENT_FULLY_DISCONNECTED OFF
  GIT_REPOSITORY "https://github.com/ifm/xmlrpc-c.git"
  GIT_TAG "f344566524281de5e6bfa2bb389e6b4038543944"
  GIT_PROGRESS ON
  GIT_SHALLOW ON
  CMAKE_CACHE_ARGS -Curl_DIR:PATH=${bin_dir}
  CMAKE_ARGS -DCMAKE_PREFIX_PATH=${bin_dir} -DCMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/third-party/downloads/xmlrpc-c"
  PATCH_COMMAND git apply  --ignore-space-change --ignore-whitespace ${CMAKE_CURRENT_SOURCE_DIR}/patches/xmlrpc.patch
)

FetchContent_Populate(xmlrpc-c )
FetchContent_GetProperties(xmlrpc-c SOURCE_DIR xmlrpc_src_dir)
FetchContent_GetProperties(xmlrpc-c BINARY_DIR xmlrpc_bin_dir)

list(APPEND CMAKE_MODULE_PATH "${xmlrpc_bin_dir}")
add_subdirectory(${xmlrpc-c_SOURCE_DIR} ${xmlrpc_bin_dir})
list(APPEND BUILTIN_INCLUDES ${xmlrpc_src_dir}/include ${xmlrpc_src_dir}/lib/util/include ${xmlrpc_bin_dir}/include  ${xmlrpc_bin_dir})
set( BUILTIN_INCLUDES ${BUILTIN_INCLUDES} PARENT_SCOPE)

FetchContent_Declare(
 glog
  GIT_REPOSITORY "https://github.com/google/glog.git"
  GIT_TAG "v0.3.5"
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/third-party/downloads/glog"
  CMAKE_ARGS -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

FetchContent_Populate(glog)
FetchContent_GetProperties(glog SOURCE_DIR glog_src_dir)
FetchContent_GetProperties(glog BINARY_DIR glog_bin_dir)

add_subdirectory(${glog_src_dir} ${glog_bin_dir})
list(APPEND BUILTIN_INCLUDES ${glog_src_dir} ${glog_bin_dir} ${glog_src_dir}/src)

FetchContent_Declare(
 pybind11
  GIT_REPOSITORY "https://github.com/pybind/pybind11"
  GIT_TAG "tags/v2.3.0"
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/third-party/downloads/pybind11"
)

FetchContent_MakeAvailable(pybind11)
FetchContent_GetProperties(pybind11 SOURCE_DIR pybind_src_dir)
FetchContent_GetProperties(pybind11 BINARY_DIR pybind_bin_dir)

list(APPEND CMAKE_MODULE_PATH "${pybind_bin_dir}")
list(APPEND BUILTIN_INCLUDES ${pybind_src_dir}/include ${pybind_bin_dir}/include  ${pybind_bin_dir})
set( BUILTIN_INCLUDES ${BUILTIN_INCLUDES} PARENT_SCOPE)

endif(BUILD_IN_DEPS)
#adding asio
add_subdirectory(asio-1-16-1)

#adding cxxopts
add_subdirectory(cxxopts-2.2.0)

#adding websocketpp
add_subdirectory(websocketpp)

