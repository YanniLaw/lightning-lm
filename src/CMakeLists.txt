include(${PROJECT_SOURCE_DIR}/cmake/ui_build.cmake)
lightning_get_ui_sources(LIGHTNING_UI_SOURCES "${LIGHTNING_WITH_PANGOLIN}")

include_directories(${PROJECT_SOURCE_DIR}/src/core/miao)
add_subdirectory(core/miao)

add_library(common SHARED
        common/nav_state.cc
        common/options.cc
)

target_link_libraries(common
        ${LIGHTNING_BASE_LIBS}
)

add_library(ui SHARED
        ${LIGHTNING_UI_SOURCES}
)

target_link_libraries(ui
        common
        ${LIGHTNING_BASE_LIBS}
)

add_library(core SHARED
        common/constant.h
        common/params.cc

        io/yaml_io.cc
        io/file_io.cc

        core/lio/eskf.cc
        core/lio/laser_mapping.cc
        core/lio/pointcloud_preprocess.cc
        core/loop_closing/loop_closing.cc

        core/g2p5/g2p5_map.cc
        core/g2p5/g2p5.cc
        core/g2p5/g2p5_subgrid.cc

        core/maps/tiled_map.cc
        core/maps/tiled_map_chunk.cc

        core/localization/localization_result.cc
        core/localization/lidar_loc/lidar_loc.cc
        core/localization/lidar_loc/pclomp/ndt_omp.cpp
        core/localization/lidar_loc/pclomp/voxel_grid_covariance_omp.cpp

        core/localization/pose_graph/pgo.cc
        core/localization/pose_graph/pgo_impl.cc
        core/localization/pose_graph/pose_extrapolator.cc
        core/localization/localization.cpp

        utils/timer.cc
        utils/pointcloud_utils.cc

        core/system/slam.cc
        core/system/loc_system.cc
        core/system/sensor_dispatcher.cc
)

target_link_libraries(core
        ${LIGHTNING_BASE_LIBS}
        common
        miao.core
        miao.utils
)

if (LIGHTNING_WITH_ROS)
    add_library(ros_bridge SHARED
            ros/sensor_bridge.cc
            ros/ros_conversions.cc
            ros/ros_visualization.cc
            ros/slam_node.cc
            ros/localization_node.cc
            wrapper/bag_io.cc
    )

    ament_target_dependencies(
            ros_bridge
            "rclcpp"
            "std_msgs"
            "geometry_msgs"
            "sensor_msgs"
            "nav_msgs"
            "std_srvs"
            "message_filters"
            "pcl_conversions"
            "tf2_ros"
            "tf2"
            "rosbag2_cpp"
    )

    add_dependencies(ros_bridge
            livox_interfaces2__rosidl_typesupport_cpp
            ${PROJECT_NAME}__rosidl_typesupport_cpp
    )

    target_link_libraries(ros_bridge
            core
            ui
            ${third_party_libs}
            livox_interfaces2__rosidl_typesupport_cpp
            ${PROJECT_NAME}__rosidl_typesupport_cpp
    )

    # Compatibility target for users that still link the old monolithic name.
    add_library(${PROJECT_NAME}.libs INTERFACE)
    target_link_libraries(${PROJECT_NAME}.libs INTERFACE core ros_bridge ui)
else ()
    add_library(${PROJECT_NAME}.libs INTERFACE)
    target_link_libraries(${PROJECT_NAME}.libs INTERFACE core ui)
endif ()

install(TARGETS common core ui miao.core miao.utils
        DESTINATION lib)

if (LIGHTNING_WITH_ROS)
    install(TARGETS ros_bridge
            DESTINATION lib)
endif ()

add_subdirectory(app)
