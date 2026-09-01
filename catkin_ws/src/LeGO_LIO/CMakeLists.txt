cmake_minimum_required(VERSION 3.0.2)
project(lego_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  tf
  nav_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED QUIET)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PCL REQUIRED pcl_common pcl_filters pcl_io)
find_package(Boost REQUIRED COMPONENTS serialization system filesystem thread date_time timer chrono)
find_package(GTSAM REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    message_runtime
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    tf
  DEPENDS PCL OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${GTSAM_DIR}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_executable(imageProjection src/imageProjection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(imageProjection PRIVATE ${PCL_CFLAGS_OTHER} -DUSE_UNORDERED_MAP=0)
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(featureAssociation src/featureAssociation.cpp)
add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(featureAssociation PRIVATE ${PCL_CFLAGS_OTHER} -DUSE_UNORDERED_MAP=0)
target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mapOptimization src/mapOptimization.cpp)
add_dependencies(mapOptimization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(mapOptimization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam ${Boost_LIBRARIES})
if(CATKIN_ENABLE_TESTING)
  add_executable(logger_test test/logger_test.cpp)
  target_link_libraries(logger_test ${catkin_LIBRARIES})
  add_test(NAME lego_lio_logger_test COMMAND logger_test)
endif()
