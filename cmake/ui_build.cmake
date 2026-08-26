function(lightning_get_ui_sources out_var with_pangolin)
    if(with_pangolin)
        set(sources
            ui/pangolin_window.cc
            ui/pangolin_window_impl.cc
            ui/ui_car.cc
            ui/ui_cloud.cc
            ui/ui_trajectory.cc)
    else()
        set(sources ui/pangolin_window_headless.cc)
    endif()

    set(${out_var} "${sources}" PARENT_SCOPE)
endfunction()
