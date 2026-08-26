function(lightning_get_architecture_compile_options out_var processor build_with_march_native)
    if(build_with_march_native)
        set(options -march=native)
    elseif(processor MATCHES "^(x86_64|AMD64|amd64)$")
        set(options
            -msse
            -msse2
            -msse3
            -msse4
            -msse4.1
            -msse4.2)
    else()
        set(options "")
    endif()

    set(${out_var} "${options}" PARENT_SCOPE)
endfunction()
