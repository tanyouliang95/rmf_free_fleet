cmake_minimum_required(VERSION 3.5.0)

project(rmf_free_fleet)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
include(GNUInstallDirs)

if(BUILD_TESTING)
  find_package(rmf_cmake_uncrustify REQUIRED)
  find_file(uncrustify_config_file NAMES "share/format/rmf_code_style.cfg")

  rmf_uncrustify(
    ARGN include src test
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
  )
endif()

set(dep_pkgs
  rclcpp
  rmf_fleet_adapter
  rmf_fleet_msgs
  rmf_task_msgs
)
foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

include_directories(
  include
  ${rmf_fleet_adapter_INCLUDE_DIRS}
)

add_executable(free_fleet_server
  src/free_fleet_server.cpp
)

target_link_libraries(free_fleet_server
  PRIVATE
    rmf_fleet_adapter::rmf_fleet_adapter
)

target_include_directories(free_fleet_server
  PRIVATE
    ${rmf_fleet_adapter_INCLUDE_DIRS}
    # ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
)


# Todo - to figure out link deps with ament
# ament_target_dependencies(free_fleet_server
#   rclcpp
#   rmf_fleet_adapter
#   rmf_fleet_msgs
# )

# -----------------------------------------------------------------------------

install(
  TARGETS free_fleet_server
  RUNTIME DESTINATION lib/rmf_free_fleet
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
