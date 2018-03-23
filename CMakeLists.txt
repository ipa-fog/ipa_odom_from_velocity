cmake_minimum_required(VERSION 2.8.3)
project(ipa_odom_from_velocity)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS nav_msgs roscpp tf)


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ipa_odom_from_velocity_node
  src/ipa_odom_from_velocity_node.cpp
  include/ipa_odom_from_velocity/ipa_odom_from_velocity_node.h
)

target_link_libraries(ipa_odom_from_velocity_node
  ${catkin_LIBRARIES}

)
