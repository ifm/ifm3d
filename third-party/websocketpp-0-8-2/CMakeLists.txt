#disabling developer warnings
add_definitions(-D-Wno-dev)
add_subdirectory(websocketpp-master)
add_library(websocketpp_wrapper INTERFACE)
	
target_include_directories(websocketpp_wrapper INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/websocketpp-master>
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

