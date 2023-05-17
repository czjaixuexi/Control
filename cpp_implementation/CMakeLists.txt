cmake_minimum_required(VERSION 3.21)
project(control)

set(CMAKE_CXX_STANDARD 14)


# include(GNUInstallDirs)
set(PACKAGE_NAME control)

# output
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


# 创建control库
add_library(control INTERFACE)
find_package(Python COMPONENTS Interpreter Development NumPy REQUIRED)
target_link_libraries(control INTERFACE
        Python::Python
        Python::Module
        Python::NumPy
        )
install(
        TARGETS control
        EXPORT install_targets
)


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


#pure_pursuit
add_executable(pure_pursuit_demo Pure_Pursuit/main.cpp  Pure_Pursuit/PurePursuit.cpp utils/VehicleModel.cpp utils/ReferenceLine.cpp)
target_link_libraries(pure_pursuit_demo PRIVATE control)

#stanley
add_executable(stanley_demo Stanley/main.cpp  Stanley/Stanley.cpp utils/VehicleModel.cpp utils/ReferenceLine.cpp)
target_link_libraries(stanley_demo PRIVATE control)


#rear_wheel_feedback
add_executable(rear_wheel_feedback_demo Rear_Wheel_Feedback/main.cpp  Rear_Wheel_Feedback/RearWheelFeedback.cpp utils/ReferenceLine.cpp utils/VehicleModel.cpp)
target_link_libraries(rear_wheel_feedback_demo PRIVATE control)