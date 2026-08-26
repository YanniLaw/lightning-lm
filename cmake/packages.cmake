find_package(glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
if (LIGHTNING_WITH_PANGOLIN)
    find_package(Pangolin REQUIRED)
    find_package(OpenGL REQUIRED)
endif ()
find_package(pcl_conversions REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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
        ${tf2_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDR_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${rosbag2_cpp_INCLUDE_DIRS}
        ${nav_msgs_INCLUDE_DIRS}
)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/livox_ros_driver/rosidl_generator_cpp
)

include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/thirdparty
)


set(third_party_libs
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        #${Pangolin_LIBRARIES}
        glog gflags
        ${yaml-cpp_LIBRARIES}
        ${pcl_conversions_LIBRARIES}
        tbb
        ${rosbag2_cpp_LIBRARIES}
)

if (LIGHTNING_WITH_PANGOLIN)
    list(APPEND third_party_libs ${Pangolin_LIBRARIES} OpenGL::GL)
endif ()
