cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(JSON_ImplicitConversions OFF) # deactivate implicit type conversion

project(common_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)  # JSON library
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include)

add_library(common_lib SHARED
  src/common_node.cpp
  src/commands.cpp)
ament_target_dependencies(common_lib rclcpp interfaces nlohmann_json)
ament_export_libraries(common_lib)
ament_export_include_directories(include)

install (
  DIRECTORY include/
  DESTINATION include
)

install (TARGETS common_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
                  test/main_test.cpp
                  test/common_node/heartbeat_test.cpp
                  test/common_node/job_finished_test.cpp
                  test/commands/jsonkeydefinition_test.cpp
                  test/commands/command_definitions_test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
          rclcpp interfaces nlohmann_json
  )
  target_link_libraries(${PROJECT_NAME}_test common_lib)
endif()

ament_package()
