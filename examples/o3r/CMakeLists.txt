##############################################################
## finding the ifm3d lib.
##############################################################
find_package(ifm3d 0.91.0 CONFIG
  REQUIRED COMPONENTS camera framegrabber image
  )
add_subdirectory(tutorials)
add_subdirectory(tutorials/configuration)
add_subdirectory(tutorials/gettingData)
add_subdirectory(tutorials/multiHead)