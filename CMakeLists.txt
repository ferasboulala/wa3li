cmake_minimum_required(VERSION 3.8)
project(wa3li)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Ofast -fno-math-errno -finline-functions -Wall -Wextra -Wpedantic -std=c++17 -std=gnu++17) #-Werror)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include/")
include_directories("${PROJECT_SOURCE_DIR}/depth2scan/")
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB SUBMODULES_PROJECT_SOURCES
    "${PROJECT_SOURCE_DIR}/depth2scan/depth2scan/*.cpp"
    "${PROJECT_SOURCE_DIR}/depth2scan/depth2scan/thirdparty/*.cpp"
)

add_executable(kinect_node src/kinect.cpp ${SUBMODULES_PROJECT_SOURCES})
target_link_libraries(kinect_node
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${FREENECT_LIBRARIES}
    -lopencv_core
    -lopencv_highgui
    -lopencv_imgproc
    -pthread
    -lfreenect)

install(TARGETS
    kinect_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
