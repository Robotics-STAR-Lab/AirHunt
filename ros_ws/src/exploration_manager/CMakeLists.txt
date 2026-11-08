cmake_minimum_required(VERSION 2.8.3)
project(exploration_manager)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
  traj_utils
  active_perception
  plan_manage
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES exploration_manager
 CATKIN_DEPENDS plan_env path_searching bspline bspline_opt active_perception traj_utils plan_manage
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


file(GLOB SOP_SOLVER_SOURCES ${PROJECT_SOURCE_DIR}/../../../third_party/sop_solver/src/*.cpp)
add_library(selective_sop src/selective_sop.cpp ${SOP_SOLVER_SOURCES})
target_include_directories(selective_sop PRIVATE ${PROJECT_SOURCE_DIR}/../../../third_party/sop_solver/include)
target_link_libraries(selective_sop pthread)

add_executable(exploration_node
  src/exploration_node.cpp
  src/fast_exploration_fsm.cpp
  src/fast_exploration_manager.cpp
  )
target_link_libraries(exploration_node selective_sop
  ${catkin_LIBRARIES}
  )
