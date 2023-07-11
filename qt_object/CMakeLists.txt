cmake_minimum_required(VERSION 3.5)
project(qt_object)

#Finds packages required for this package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)

#Set the C++ standard to use
set(CMAKE_CXX_STANDARD 17)

#Auto-generate the Qt Meta-Object Compiler
set(CMAKE_AUTOMOC ON)

#Defines the paths for the "#include" command
include_directories(src)

#Adds the class definitions to a common library
add_library(${PROJECT_NAME} SHARED
    src/qt_object.cpp
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
    Qt5
)

#Creates the executable based on the main loop
add_executable(qt_object_node nodes/main.cpp)
target_link_libraries(qt_object_node ${PROJECT_NAME})

#Specifies installation rules
install(
    TARGETS ${PROJECT_NAME} qt_object_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#Defines export rules for the ament_cmake build system
ament_export_include_directories(src)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp Qt5)

#Builds the package
ament_package()