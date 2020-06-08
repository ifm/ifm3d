if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W2")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "[Cc]lang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra -Wshadow")
endif()

add_library(cxxopts INTERFACE)
	
target_include_directories(cxxopts INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cxxopts/include>
)