find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
if (LIGHTNING_WITH_PANGOLIN)
    find_package(Pangolin REQUIRED)
    find_package(OpenGL REQUIRED)
endif ()
find_package(OpenCV REQUIRED)

if (LIGHTNING_WITH_ROS)
    find_package(pcl_conversions REQUIRED)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(message_filters REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(rosbag2_cpp REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
endif ()

# OMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

include(${PROJECT_SOURCE_DIR}/cmake/architecture_flags.cmake)
lightning_get_architecture_compile_options(
        LIGHTNING_ARCHITECTURE_COMPILE_OPTIONS
        "${CMAKE_SYSTEM_PROCESSOR}"
        "${BUILD_WITH_MARCH_NATIVE}")
add_compile_options(${LIGHTNING_ARCHITECTURE_COMPILE_OPTIONS})

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIRS}
)

if (LIGHTNING_WITH_ROS)
    include_directories(
            ${tf2_INCLUDE_DIRS}
            ${pcl_conversions_INCLUDE_DIRS}
            ${rclcpp_INCLUDE_DIRS}
            ${rosbag2_cpp_INCLUDE_DIRS}
            ${nav_msgs_INCLUDE_DIRS}
    )
endif ()

if (LIGHTNING_WITH_ROS)
    include_directories(
            ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/livox_ros_driver/rosidl_generator_cpp
    )
endif ()

include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/thirdparty
)


set(LIGHTNING_BASE_LIBS
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        glog gflags
        ${yaml-cpp_LIBRARIES}
        tbb
)

if (LIGHTNING_WITH_PANGOLIN)
    list(APPEND LIGHTNING_BASE_LIBS ${Pangolin_LIBRARIES} OpenGL::GL)
endif ()

set(third_party_libs ${LIGHTNING_BASE_LIBS})
if (LIGHTNING_WITH_ROS)
    list(APPEND third_party_libs
            ${pcl_conversions_LIBRARIES}
            ${rosbag2_cpp_LIBRARIES}
    )
endif ()
