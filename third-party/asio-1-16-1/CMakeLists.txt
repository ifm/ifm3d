add_library(asio INTERFACE)
target_compile_definitions(asio INTERFACE -DASIO_STANDALONE)
#used for tracing the asio async calls
#target_compile_definitions(asio INTERFACE -DASIO_ENABLE_HANDLER_TRACKING)
target_include_directories(asio SYSTEM INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/asio/include>
)
