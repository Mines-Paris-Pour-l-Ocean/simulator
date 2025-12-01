cmake_minimum_required(VERSION 3.5)
project(battery)

# Standard C++17 (comme dans votre Makefile)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Options de compilation
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O0)
endif()

# Définir NDEBUG
add_definitions(-DNDEBUG)

# Répertoires d'inclusion
include_directories(
  /usr/include
  /usr/local/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Trouver les bibliothèques GMP et MPFR
find_library(GMP_LIBRARY gmp REQUIRED)
find_library(MPFR_LIBRARY mpfr REQUIRED)

# Packages ROS2
find_package(ament_cmake REQUIRED)
find_package(stonefish_ros2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Precompiled header (optionnel avec CMake moderne)
# CMake 3.16+ supporte target_precompile_headers
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
  add_library(pch INTERFACE)
  target_precompile_headers(pch INTERFACE src/battery_without_charge/pch.h)
endif()

# Bibliothèque pour les fichiers communs
add_library(battery_lib
  src/battery_without_charge/M_function.cpp
  src/battery_without_charge/setup.cpp
  src/battery_without_charge/battery.cpp
)

target_include_directories(battery_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(battery_lib
  ${GMP_LIBRARY}
  ${MPFR_LIBRARY}
)

# Lier le precompiled header si disponible
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.16)
  target_link_libraries(battery_lib pch)
endif()

# Exécutable principal
add_executable(battery_state src/battery_state.cpp)

# Dépendances ROS2
ament_target_dependencies(battery_state 
  rclcpp 
  std_msgs 
  stonefish_ros2
)

# Lier la bibliothèque commune et les libs externes
target_link_libraries(battery_state
  battery_lib
  ${GMP_LIBRARY}
  ${MPFR_LIBRARY}
)

# Installation
install(TARGETS
  battery_state
  DESTINATION lib/${PROJECT_NAME})

ament_package()
