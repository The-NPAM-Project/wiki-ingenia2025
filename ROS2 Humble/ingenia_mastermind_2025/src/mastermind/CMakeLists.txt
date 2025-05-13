cmake_minimum_required(VERSION 3.8)
project(mastermind)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# Para los nodos en Python
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


# Incluir archivos de cabecera .hpp
include_directories(include)

# Añadir archivos ejecutables
add_executable(master
    src/master.cpp
)
add_executable(bed_controller
    src/bed_controller.cpp
)
add_executable(calibration
    src/calibration.cpp
)

# Especificar dependencias de nodos
ament_target_dependencies(master
    rclcpp
    std_msgs
)
ament_target_dependencies(bed_controller
    rclcpp
)
ament_target_dependencies(calibration
    rclcpp
)

# Para que ROS2 encuentre los ejecutables
install(TARGETS
    master
    bed_controller
    calibration
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
    scripts/interface.py
    DESTINATION lib/${PROJECT_NAME}
)


ament_package()
