cmake_minimum_required(VERSION 3.5)

project(dh_pgc140_urdf)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)
