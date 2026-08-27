cmake_minimum_required(VERSION 3.8)
project(piper_celda_moveit)
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/publicar_escena_celda.py
  scripts/manipular_laminilla.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
