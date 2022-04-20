cmake_minimum_required(VERSION 3.1)
project(global_path_planning)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)
if(NOT Boost_FOUND)
    message("Not found Boost")
endif()

catkin_python_setup()

include_directories(${Boost_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    costmap_2d
    geometry_msgs
    map_msgs
    move_base_msgs
    nav_msgs
    roscpp
    std_msgs
    tf
    visualization_msgs
)
find_package(Boost)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
  # src/bspline.cpp
  src/KDTree/KDTree.cpp
)
add_library(dynamicvoronoi src/dynamic_voronoi/dynamicvoronoi.cpp)
add_executable(global_path_planning_
  src/global_path_planning.cpp
)
add_dependencies(global_path_planning_ ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_path_planning_ dynamicvoronoi ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})




add_executable(example src/dynamic_voronoi/example.cpp)
target_link_libraries(example dynamicvoronoi)


#############
## Install ##
#############

# install nodes
install(TARGETS global_path_planning
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  # test all launch files
  roslaunch_add_file_check(launch)
endif()
