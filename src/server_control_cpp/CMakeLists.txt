cmake_minimum_required(VERSION 3.8)
project(server_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(create_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)


# Use pkg-config to find libmicrohttpd
find_package(PkgConfig REQUIRED)
pkg_check_modules(MHD REQUIRED libmicrohttpd)

# Adicionar executável
add_executable(server_control_cpp src/server_control_cpp.cpp)

# Linka as bibliotecas necessárias
target_link_libraries(server_control_cpp 
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${create_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nlohmann_json_LIBRARIES}
  ${MHD_LIBRARIES}  # Linka a libmicrohttpd
)

# Inclui os diretórios necessários
target_include_directories(server_control_cpp
  PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${create_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
  ${MHD_INCLUDE_DIRS}  # Inclui os diretórios de headers da libmicrohttpd
)

# Instalar o executável
install(TARGETS
  server_control_cpp
  DESTINATION lib/${PROJECT_NAME})

# Verificações de linter (se necessário)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
