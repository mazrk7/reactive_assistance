cmake_minimum_required(VERSION 2.8.3)
project(reactive_assistance)

add_compile_options(-std=c++11 -O3 -Wall)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    pcl_ros
    roscpp
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    DEPENDS Boost
    INCLUDE_DIRS include
    LIBRARIES reactive_assistance
    CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros roscpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/dist_util.cpp
    src/obstacle_avoidance.cpp
    src/obstacle_map.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/reactive_assistance_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)