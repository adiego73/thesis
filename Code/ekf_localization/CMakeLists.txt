cmake_minimum_required(VERSION 2.8.3)
project(ekf_localization)

set(CATKIN_ENABLE_TESTING 0)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        mavros_msgs
        nav_msgs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        message_generation
        message_filters
        poles_vision
        octomap_msgs
        )
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system)
# YAML-CPP can be installed following the instructions here https://github.com/jbeder/yaml-cpp
find_package(yaml-cpp REQUIRED)
find_package(octomap REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        QRCodePose.msg
        QRCodeStamped.msg
        QRCodeStampedArray.msg
        RangeAndBearingPole.msg
        StringStamped.msg
)

add_service_files(
        FILES
        GetDroneState.srv
        GetMarkerState.srv
        SaveMap.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        octomap_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp sensor_msgs std_msgs message_runtime
)

# add yaml-cpp library to catkin libraries.
set(catkin_LIBRARIES ${catkin_LIBRARIES} yaml-cpp ${OCTOMAP_LIBRARIES})

###########
## Build ##
###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${OCTOMAP_INCLUDE_DIRS}
)

# ekf_localization_node
add_executable(${PROJECT_NAME}_node
        src/ekf_localization_node.cpp
        src/ekf_slam.cpp
        src/commons.cpp
        src/pole.cpp
        src/marker.cpp
        src/map_manager.cpp
        src/range.cpp
        include/ekf_localization/landmark.hpp
        include/ekf_localization/pole.hpp
        include/ekf_localization/marker.hpp
        include/ekf_localization/ekf_slam.hpp
        include/ekf_localization/ekf.hpp
        include/ekf_localization/commons.hpp
        include/ekf_localization/map_manager.hpp
        include/ekf_localization/range.hpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})


# tf_qr_code_node
add_executable(qr_code_node src/qr_code_node.cpp src/commons.cpp)
add_dependencies(qr_code_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(qr_code_node ${catkin_LIBRARIES})

if (CMAKE_BUILD_TYPE MATCHES Debug)
    target_compile_definitions(${PROJECT_NAME}_node PUBLIC DEBUG)
    target_compile_definitions(qr_code_node PUBLIC DEBUG)

    ## fake pole localization
    add_executable(fake_pole_localization src/fake_pole_localization.cpp src/commons.cpp)
    add_dependencies(fake_pole_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(fake_pole_localization ${catkin_LIBRARIES})

    # fake marker localization
    add_executable(fake_marker_localization src/fake_marker_localization.cpp src/commons.cpp)
    add_dependencies(fake_marker_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(fake_marker_localization ${catkin_LIBRARIES})
endif ()

catkin_install_python(PROGRAMS
        scripts/build_plots.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )