cmake_minimum_required(VERSION 3.2)
project(cartographer)
set(CMAKE_CXX_STANDARD 17)
include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
google_initialize_cartographer_project()
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package()
include(FindPkgConfig)
find_library(CAIRO_LIBRARIES cairo)
install(FILES package.xml DESTINATION share/cartographer)
install(DIRECTORY cmake DESTINATION share/cartographer)
file(GLOB_RECURSE ALL_LIBRARY_HDRS "cartographer/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "cartographer/*.cc")
file(GLOB_RECURSE TEST_MAIN "cartographer/test_node.cc")
file(GLOB_RECURSE SLAM_MAIN "cartographer/slam_node.cc")
file(GLOB_RECURSE TEST_HDRS "cartographer/*_test.h")
file(GLOB_RECURSE TEST_SRCS "cartographer/*_test.cc")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${TEST_MAIN} ${SLAM_MAIN} ${TEST_SRCS} )
set(INSTALL_SOURCE_HDRS ${ALL_LIBRARY_HDRS})
file(GLOB_RECURSE INTERNAL_HDRS "cartographer/*/internal/*.h")
list(REMOVE_ITEM INSTALL_SOURCE_HDRS ${INTERNAL_HDRS})
file(GLOB_RECURSE ALL_PROTOS "cartographer/*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")
  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()
set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_PROTO_SRCS})
set(INSTALL_GENERATED_HDRS ${ALL_PROTO_HDRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} /opt/ros/noetic/include ${OpenCV_INCLUDE_DIRS} ${GTest_INCLUDE_DIRS} ${Glog_INCLUDE_DIRS} 
${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIR})


if(${BUILD_TEST})
  add_executable(test_node ${TEST_MAIN} ${ALL_LIBRARY_SRCS} ${TEST_HDRS} ${TEST_SRCS})
  target_link_libraries(test_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES} ${PROTOBUF_LIBRARY}
    ${Boost_LIBRARIES} ${CAIRO_LIBRARIES} pthread gtest glog gflags)
endif()


if(${BUILD_SLAM})
  add_executable(slam_node ${SLAM_MAIN} ${ALL_LIBRARY_SRCS})
  target_link_libraries(slam_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES} ${PROTOBUF_LIBRARY}
    ${Boost_LIBRARIES} ${CAIRO_LIBRARIES} pthread glog gflags )
endif()

foreach(HDR ${INSTALL_SOURCE_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

foreach(HDR ${INSTALL_GENERATED_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()