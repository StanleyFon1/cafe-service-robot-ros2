cmake_minimum_required(VERSION 3.8)
project(serv_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
  )
  
  ament_package()
