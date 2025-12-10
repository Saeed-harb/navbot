cmake_minimum_required(VERSION 3.8)
project(navbot_libraries)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# let the compiler search for headers in the include folder
include_directories(
  include
  /home/ts/Libraries/NumCpp/include
)

# define a library target called navbot_path_planning
add_library(navbot_libraries src/path_finding.cpp) 

# this line to exports the library
ament_export_targets(navbot_libraries HAS_LIBRARY_TARGET)

# install the include/navbot_libraries directory to the install/include/navbot_libraries
install(
  DIRECTORY include/navbot_libraries
  DESTINATION include
)

install(
  TARGETS navbot_libraries
  EXPORT navbot_libraries
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
