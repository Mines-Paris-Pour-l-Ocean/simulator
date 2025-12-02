cmake_minimum_required(VERSION 3.5)
project(battery)

# Standard C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Options de compilation
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O0)
endif()

# Définir NDEBUG
add_definitions(-DNDEBUG)

# Packages ROS2 (AVANT rosidl_generate_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stonefish_ros2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Générer les interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CurrentInfos.msg"
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION include/${PROJECT_NAME}
)


# Trouver les bibliothèques GMP et MPFR
find_library(GMP_LIBRARY gmp REQUIRED)
find_library(MPFR_LIBRARY mpfr REQUIRED)

# Répertoires d'inclusion
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Bibliothèque pour les fichiers communs
add_library(battery_lib SHARED
  src/battery/M_function.cpp
  src/battery/setup.cpp
  src/battery/battery.cpp
)


target_include_directories(battery_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(battery_lib
  ${GMP_LIBRARY}
  ${MPFR_LIBRARY}
)

ament_target_dependencies(battery_lib
 rclcpp 
 ament_index_cpp 
 std_srvs
 std_msgs
)

ament_export_libraries(battery_lib)
ament_export_include_directories(include)

# Exécutable principal
add_executable(battery_state src/battery_state.cpp)


target_include_directories(battery_state PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Lier la bibliothèque commune et les libs externes
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(battery_state
  battery_lib
  ${GMP_LIBRARY}
  ${MPFR_LIBRARY}
	${cpp_typesupport_target}
)

# Dépendances ROS2
ament_target_dependencies(battery_state 
  rclcpp 
  std_msgs 
	sensor_msgs
  stonefish_ros2
)


# Installation
install(
	TARGETS battery_lib
	EXPORT export_${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(TARGETS
  battery_state
  DESTINATION lib/${PROJECT_NAME})

ament_package()
