cmake_minimum_required(VERSION 3.5)
project(multirobot_map_merge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

if("${OpenCV_VERSION}" VERSION_LESS "3.0")
  message(FATAL_ERROR "This package needs OpenCV >= 3.0")
endif()
if("${OpenCV_VERSION}" VERSION_LESS "4.0")
  message(WARNING "This package supports OpenCV 3, but some features may not be available. Upgrade to OpenCV 4 to take advantage of all features.")
endif()

set(DEPENDENCIES
  rclcpp
  geometry_msgs
  image_geometry
  map_msgs
  nav_msgs
  tf2_geometry_msgs
  tf2
  tf2_ros
  OpenCV
)

include_directories(
  include
)

install(
  DIRECTORY include/map_merge/
  DESTINATION include/map_merge/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

add_library(combine_grids STATIC
  src/combine_grids/grid_compositor.cpp
  src/combine_grids/grid_warper.cpp
  src/combine_grids/merging_pipeline.cpp
)

add_executable(map_merge
  src/map_merge.cpp
)

target_include_directories(map_merge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(combine_grids PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(combine_grids ${rclcpp_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(map_merge combine_grids)

ament_target_dependencies(map_merge ${DEPENDENCIES})
ament_target_dependencies(combine_grids ${DEPENDENCIES})

install(
  TARGETS combine_grids
  EXPORT export_combine_grids
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS map_merge
  DESTINATION lib/${PROJECT_NAME}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

ament_export_include_directories(include)
ament_export_libraries(combine_grids)
ament_package()

