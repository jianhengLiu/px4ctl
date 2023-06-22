cmake_minimum_required(VERSION 3.0.2)
project(px4ctl)

set(CMAKE_BUILD_TYPE "Release")

# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set(CMAKE_BUILD_TYPE "Debug")
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge

        # dynamixel_workbench_toolbox
)

find_package(OpenCV REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Eigen3)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

catkin_package()

# ################## source and driver ######################
add_executable(tag
        src/tag_det.cpp)
target_link_libraries(tag
        ${OpenCV_LIBS})

add_executable(tag_aided_loc_node src/tag_aided_loc_node.cpp)
target_link_libraries(tag_aided_loc_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(tag_servo_node src/tag_servo_node.cpp)
target_link_libraries(tag_servo_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(auto_catch_ctl src/auto_catch_ctl.cpp)
target_link_libraries(auto_catch_ctl ${catkin_LIBRARIES} ${OpenCV_LIBS})

# # require dynamixel_workbench_toolbox dependency
# add_executable(nbv_ctl src/nbv_ctl.cpp)
# target_link_libraries(nbv_ctl ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(simple_cmd_ctl src/simple_cmd_ctl.cpp)
target_link_libraries(simple_cmd_ctl ${catkin_LIBRARIES} ${OpenCV_LIBS})
