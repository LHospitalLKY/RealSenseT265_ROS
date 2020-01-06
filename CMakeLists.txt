cmake_minimum_required(VERSION 2.4.6)
project(RealSense)

set(CMAKE_BUILD_TYPE "Debug")

# ros
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
)
if(NOT catkin_FOUND)
    message("NOT FOUND Catkin!")
endif()
# opencv
find_package(OpenCV 3 REQUIRED)
if(NOT OpenCV_FOUND)
    message("NOT FOUND OpenCV!")
endif()
# realsense
find_package(realsense2 REQUIRED)
if(NOT realsense2_FOUND)
    message("NOT FOUND REALSENSE!")
endif()
# gtest
find_package(GTest)
find_package(Threads REQUIRED)

include_directories(
    "/usr/include/eigen3/"
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(${PROJECT_NAME}_node
    include/RealSense/RealSenseT265_node.h
    src/RealSenseT265_node.cpp
    src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    glog
)

add_executable(callback_TEST
    Test/callback_TEST.cpp
)
target_link_libraries(callback_TEST
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
    glog
)

add_executable(context_TEST
    Test/context.cpp
)
target_link_libraries(context_TEST
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
    glog
)

add_executable(sync_imu_TEST
    Test/sync_imu_TEST.cpp
)
target_link_libraries(sync_imu_TEST
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
    glog
)

add_executable(RealSenseT265_NODE_TEST
    include/RealSense/RealSenseT265_node.h
    src/RealSenseT265_node.cpp
    Test/RealSenseT265_node_TEST.cpp
)
target_link_libraries(RealSenseT265_NODE_TEST
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${realsense2_LIBRARY}
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    glog
)
