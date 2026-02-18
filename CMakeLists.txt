cmake_minimum_required(VERSION 3.8)
project(mvp_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(GTSAM REQUIRED)
find_package(GTSAM)
if(GTSAM_FOUND)
  message(STATUS "Found GTSAM ${GTSAM_VERSION}")
else()
  message(STATUS "GTSAM Not Found")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

set(library_name mvp_localization)
set(executable_name mvp_localization_node)


set(dependencies
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

####################################################################
#####                      setup compiling                     #####
####################################################################

include_directories(
  include
  /usr/local/include
  ${EIGEN3_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}

)

link_directories(/usr/local/lib) # GTSAM library path

# Library
# add_library(${library_name} SHARED
#   src/mvp_localization/gps_processor.cpp
#   src/mvp_localization/imu_processor.cpp
#   src/mvp_localization/mvp_localization.cpp
# )


# ament_target_dependencies(${library_name}
  # ${dependencies}
# )

# target_link_libraries( 
#   ${library_name}
#   gtsam
#   gtsam_unstable
#   ${GeographicLib_LIBRARIES}
#   # ${GTSAM_LIBRARIES}
# )

add_executable(mvp_localization_node
  src/main.cpp
)

target_link_libraries(${executable_name} 
  # ${library_name}
  gtsam
  # gtsam_unstable
  # PRIVATE
  Eigen3::Eigen
  ${GeographicLib_LIBRARIES}
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

####################################################################
#####                       setup install                      #####
####################################################################


# Install
install(TARGETS ${executable_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(DIRECTORY launch 
#   DESTINATION share/${PROJECT_NAME}
# )

# install(DIRECTORY config 
#   DESTINATION share/${PROJECT_NAME}
# )


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()


