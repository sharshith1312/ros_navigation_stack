

cmake_minimum_required(VERSION 3.22.1)
project(mobile_base_description LANGUAGES C CXX)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()




ament_auto_package(INSTALL_TO_SHARE meshes urdf launch rviz)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}/
)
