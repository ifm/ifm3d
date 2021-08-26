cmake_minimum_required(VERSION 3.16)

##############################################################
## finding the ifm3d lib.
##############################################################
find_package(ifm3d CONFIG
  REQUIRED COMPONENTS camera framegrabber image
  )
add_subdirectory(tutorials/configuration)
add_subdirectory(tutorials/gettingData)
add_subdirectory(tutorials/multiHead)