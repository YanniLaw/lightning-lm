cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED LIGHTNING_SOURCE_DIR)
    message(FATAL_ERROR "LIGHTNING_SOURCE_DIR must point to the repository root")
endif()

include("${LIGHTNING_SOURCE_DIR}/cmake/architecture_flags.cmake")
include("${LIGHTNING_SOURCE_DIR}/cmake/ui_build.cmake")

function(assert_list_equal actual_var expected_var description)
    if(NOT "${${actual_var}}" STREQUAL "${${expected_var}}")
        message(FATAL_ERROR
            "${description}\n"
            "  expected: ${${expected_var}}\n"
            "  actual:   ${${actual_var}}")
    endif()
endfunction()

lightning_get_architecture_compile_options(arm_options "aarch64" FALSE)
set(expected_arm_options "")
assert_list_equal(arm_options expected_arm_options
    "aarch64 builds must not receive x86-only compiler options")

lightning_get_architecture_compile_options(x86_options "x86_64" FALSE)
set(expected_x86_options
    -msse
    -msse2
    -msse3
    -msse4
    -msse4.1
    -msse4.2)
assert_list_equal(x86_options expected_x86_options
    "x86_64 builds must retain the existing SSE optimization options")

lightning_get_architecture_compile_options(native_options "aarch64" TRUE)
set(expected_native_options -march=native)
assert_list_equal(native_options expected_native_options
    "native builds must explicitly request the host architecture")

lightning_get_ui_sources(headless_sources FALSE)
set(expected_headless_sources ui/pangolin_window_headless.cc)
assert_list_equal(headless_sources expected_headless_sources
    "headless builds must use the Pangolin-free window implementation")

lightning_get_ui_sources(pangolin_sources TRUE)
set(expected_pangolin_sources
    ui/pangolin_window.cc
    ui/pangolin_window_impl.cc
    ui/ui_car.cc
    ui/ui_cloud.cc
    ui/ui_trajectory.cc)
assert_list_equal(pangolin_sources expected_pangolin_sources
    "UI builds must retain the original Pangolin implementation")

message(STATUS "Portability CMake tests passed")
