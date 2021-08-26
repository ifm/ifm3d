add_executable(getting_data_ex getting_data_ex.cpp)
target_link_libraries(getting_data_ex
                      ifm3d::camera
                      ifm3d::framegrabber
                      ifm3d::image
                      )