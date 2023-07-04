cmake_minimum_required(VERSION 3.5)
project(interface_node_demo)

#Finds packages required for this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)

#Set the C++ standard to use
set(CMAKE_CXX_STANDARD 17)

#Auto-generate the Qt Meta-Object Compiler
set(CMAKE_AUTOMOC ON)

#Defines the paths for the "#include" command
include_directories(src)

#Adds the class definitions to a common library
add_library(${PROJECT_NAME} SHARED
    src/external_qt.cpp
    src/external_ros.cpp
    src/interface.cpp
)

#Includes necessary directories for the Q_OBJECT macro compilation
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

#Links libraries to the project target
target_link_libraries(${PROJECT_NAME}
    Qt5::Core
)

#Declares the dependencies to the ament_cmake build system
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    Qt5
)

#Creates the executable based on the main loop
add_executable(external_ros nodes/external_ros_node.cpp)
target_link_libraries(external_ros ${PROJECT_NAME})

# Not used ! Requires to remove the interface executable and modifications in external_qt.h and external_qt.cpp. See external_qt_node.cpp
# add_executable(external_qt nodes/external_qt_node.cpp)
# target_link_libraries(external_qt ${PROJECT_NAME})

#Creates the executable based on the main loop
add_executable(interface nodes/interface_node.cpp)
target_link_libraries(interface ${PROJECT_NAME})

#Specifies installation rules
install(
    TARGETS ${PROJECT_NAME} external_ros interface #external_qt
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#Defines export rules for the ament_cmake build system
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs Qt5)

#Builds the package
ament_package()