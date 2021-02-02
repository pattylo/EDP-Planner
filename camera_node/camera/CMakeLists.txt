cmake_minimum_required(VERSION 3.0.2)
project(camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(trial 
src/trial.cpp
src/include/run_yolo.cpp)

target_link_libraries(trial
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
   )

add_executable(yolotiny
src/yolotiny.cpp
src/include/run_yolo.cpp)

target_link_libraries(yolotiny
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
   )
