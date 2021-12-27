cmake_minimum_required(VERSION 3.8)
project(wa3li)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -std=c++17 -std=gnu++17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs)

find_package(OpenCV REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include/wa3li/")
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(kinect_node src/kinect.cpp)
target_link_libraries(kinect_node ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES} ${OpenCV_LIBRARIES} ${FREENECT_LIBRARIES}
    -lopencv_core -lopencv_highgui -lopencv_imgproc -pthread -lfreenect)

install(TARGETS
    kinect_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
