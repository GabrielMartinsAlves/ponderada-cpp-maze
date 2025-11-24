cmake_minimum_required(VERSION 3.10)
project(cg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

add_executable(cg_autonomous_navigator cg_autonomous_navigator.cpp)
ament_target_dependencies(cg_autonomous_navigator rclcpp cg_interfaces)
install(TARGETS cg_autonomous_navigator DESTINATION lib/${PROJECT_NAME})

# Instala o script Python maze.py como executável ROS2
# Instala o script Python maze.py e o launcher maze
install(PROGRAMS maze.py maze
	DESTINATION lib/${PROJECT_NAME}
)
# Instala o pacote Python cg/cg para que os imports funcionem
install(DIRECTORY cg/
	DESTINATION lib/python3.13/site-packages/cg
	FILES_MATCHING PATTERN "*.py"
)
# Instala o diretório de mapas para acesso pelo jogo
install(DIRECTORY maps
	DESTINATION share/${PROJECT_NAME}
)
# Instala o diretório lobotomy_kaisen para acesso pelo jogo
install(DIRECTORY lobotomy_kaisen
	DESTINATION share/${PROJECT_NAME}
)

ament_package()