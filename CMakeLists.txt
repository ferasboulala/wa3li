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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(wa3li_protocol REQUIRED)
find_package(libfreenect REQUIRED)

add_subdirectory(depth2scan)
add_subdirectory(slam)
add_subdirectory(kobuki)

macro(app progname source)
    add_executable(${progname} ${source} apps/thirdparty/log.cpp)
    target_include_directories(${progname} PRIVATE ${OpenCV_INCLUDE_DIRS} depth2scan)
    target_link_libraries(${progname} ${FREENECT_LIBRARIES} ${OpenCV_LIBRARIES}
        -lopencv_core -lopencv_highgui -lopencv_imgproc depth2scan)
endmacro()

macro(app progname source)
    add_executable(${progname} ${source})
    target_include_directories(${progname} PRIVATE ${OpenCV_INCLUDE_DIRS} depth2scan slam include kobuki)
    target_link_libraries(${progname}
        ${rclcpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${nav_msgs_LIBRARIES}
        ${tf2_LIBRARIES}
        ${tf2_ros_LIBRARIES}
        ${wa3li_protocol_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${FREENECT_LIBRARIES}
        depth2scan
        slam
        kobuki
        -lopencv_core
        -lopencv_highgui
        -lopencv_imgproc
        -pthread
        -lfreenect
        -lfreenect_sync)
    ament_target_dependencies(${progname} wa3li_protocol tf2 tf2_ros)
endmacro()

app(kinect_node src/kinect.cpp)
app(kobuki_node src/kobuki.cpp)
app(nav_node src/nav.cpp)

install(TARGETS
    kinect_node
    kobuki_node
    nav_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
