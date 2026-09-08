if (LIGHTNING_WITH_ROS)
    add_executable(run_frontend_offline run_frontend_offline.cc)
    add_executable(run_loop_offline run_loop_offline.cc)
    add_executable(run_slam_offline run_slam_offline.cc)
    add_executable(run_slam_online run_slam_online.cc)
    add_executable(run_loc_offline run_loc_offline.cc)
    add_executable(run_loc_online run_loc_online.cc)

    foreach (target_name IN ITEMS
            run_frontend_offline
            run_loop_offline
            run_slam_offline
            run_slam_online
            run_loc_offline
            run_loc_online)
        target_link_libraries(${target_name} core ros_bridge ui)
    endforeach ()

    if (LIGHTNING_WITH_PANGOLIN)
        add_executable(test_ui test_ui.cc)
        target_link_libraries(test_ui core ui)
    endif ()

    install(TARGETS
            run_frontend_offline
            run_loop_offline
            run_slam_offline
            run_slam_online
            run_loc_offline
            run_loc_online
            RUNTIME DESTINATION lib/${PROJECT_NAME})
endif ()
