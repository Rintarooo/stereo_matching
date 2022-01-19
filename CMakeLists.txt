cmake_minimum_required(VERSION 3.0.2)
project(stereo_matching)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")# -g3
find_package(OpenCV 3.4.11 REQUIRED)
find_package(PCL 1.11.0 REQUIRED)
find_package(Eigen3 REQUIRED)

### directories where header files exists
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

cuda_add_executable(main
  main.cpp
)
  # OPTIONS -G -g


target_link_libraries(main
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

# for debug
message(WARNING "prefix ${CMAKE_PREFIX_PATH}")    

message("-- OpenCV --")
message(WARNING "version ${OpenCV_VERSION}")
message(WARNING "install path ${OpenCV_INSTALL_PATH}") 
message(WARNING "config path ${OpenCV_CONFIG_PATH}") # look at the output of this message
message(WARNING "libs ${OpenCV_LIBS}")
message(WARNING "include dirs ${OpenCV_INCLUDE_DIRS}")

message("-- PCL --")
message(WARNING "version ${PCL_VERSION}")
message(WARNING "where libs ${PCL_LIBRARY_DIRS}")
message(WARNING "libs ${PCL_LIBRARIES}")
message(WARNING "components ${PCL_COMPONENTS}")
message(WARNING "include dirs ${PCL_INCLUDE_DIRS}")