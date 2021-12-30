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
find_package(nav_msgs REQUIRED)

find_package(OpenCV REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include/")
include_directories("${PROJECT_SOURCE_DIR}/depth2scan/")
include_directories("${PROJECT_SOURCE_DIR}/kobuki/")
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB SUBMODULES_PROJECT_SOURCES
    "${PROJECT_SOURCE_DIR}/depth2scan/depth2scan/*.cpp"
    "${PROJECT_SOURCE_DIR}/kobuki/kobuki/*.cpp"
)

macro(app progname source)
    add_executable(${progname} ${source} ${SUBMODULES_PROJECT_SOURCES})
    target_link_libraries(${progname}
        ${rclcpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${FREENECT_LIBRARIES}
        -lopencv_core
        -lopencv_highgui
        -lopencv_imgproc
        -pthread
        -lfreenect
        -lfreenect_sync)
endmacro()

app(kinect_node src/kinect.cpp)
app(kobuki_node src/kobuki.cpp)

install(TARGETS
    kinect_node
    kobuki_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
