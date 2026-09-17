if (NOT DEFINED PROJECT_IS_TOP_LEVEL OR PROJECT_IS_TOP_LEVEL)
    find_program(pkg-bin pkg HINTS /opt/pkg)
    if (pkg-bin)
        message(STATUS "found pkg ${pkg-bin}")
    else ()
        set(pkg-bin "${CMAKE_BINARY_DIR}/dl/pkg")
        if (${CMAKE_SYSTEM_NAME} STREQUAL "Linux" AND ${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "aarch64")
            set(pkg-url "pkg-linux-arm64")
        elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
            set(pkg-url "pkg")
        elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
            set(pkg-url "pkg.exe")
        elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
            set(pkg-url "pkgosx")
        else ()
            message(STATUS "Not downloading pkg tool. Using pkg from PATH.")
            set(pkg-bin "pkg")
        endif ()

        if (pkg-url)
            if (NOT EXISTS ${pkg-bin})
                message(STATUS "Downloading pkg binary from https://github.com/motis-project/pkg/releases/latest/download/${pkg-url}")
                file(DOWNLOAD "https://github.com/motis-project/pkg/releases/latest/download/${pkg-url}" ${pkg-bin})
                if (UNIX)
                    execute_process(COMMAND chmod +x ${pkg-bin})
                endif ()
            else ()
                message(STATUS "Pkg binary located in project.")
            endif ()
        endif ()
    endif ()

    if (DEFINED ENV{GITHUB_ACTIONS})
        # pkg's fast path validates dependency commits, but not the generated
        # deps/CMakeLists.txt in the runner's shared dependency cache.
        # Rebuild that integration file from the manifest on every CI run.
        file(REMOVE "${CMAKE_SOURCE_DIR}/.pkg.lock")
        message(STATUS "${pkg-bin} -l -h -f")
        execute_process(
                COMMAND ${pkg-bin} -l -h -f
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                RESULT_VARIABLE pkg-result
        )
    else ()
        message(STATUS "${pkg-bin} -l")
        execute_process(
                COMMAND ${pkg-bin} -l
                WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
                RESULT_VARIABLE pkg-result
        )
    endif ()

    if (NOT pkg-result EQUAL 0)
        message(FATAL_ERROR "pkg failed: ${pkg-result}")
    endif ()

    # TinyIntIDFunc::set performs a read-modify-write, so its packed storage
    # must be initialized before individual entries are assigned.
    set(ifc-source-dir "${CMAKE_CURRENT_SOURCE_DIR}/deps/InertialFlowCutter")
    set(ifc-init-patch
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/patches/inertialflowcutter-initialize-tiny-int.patch")
    if (EXISTS "${ifc-source-dir}")
        execute_process(
                COMMAND git apply --unidiff-zero --check "${ifc-init-patch}"
                WORKING_DIRECTORY "${ifc-source-dir}"
                RESULT_VARIABLE ifc-patch-check-result
                OUTPUT_QUIET
                ERROR_QUIET
        )
        if (ifc-patch-check-result EQUAL 0)
            execute_process(
                    COMMAND git apply --unidiff-zero "${ifc-init-patch}"
                    WORKING_DIRECTORY "${ifc-source-dir}"
                    RESULT_VARIABLE ifc-patch-result
            )
            if (NOT ifc-patch-result EQUAL 0)
                message(FATAL_ERROR "Failed to patch InertialFlowCutter: ${ifc-patch-result}")
            endif ()
        else ()
            execute_process(
                    COMMAND git apply --unidiff-zero --reverse --check "${ifc-init-patch}"
                    WORKING_DIRECTORY "${ifc-source-dir}"
                    RESULT_VARIABLE ifc-reverse-patch-check-result
                    OUTPUT_QUIET
                    ERROR_QUIET
            )
            if (NOT ifc-reverse-patch-check-result EQUAL 0)
                message(FATAL_ERROR "InertialFlowCutter initialization patch does not apply")
            endif ()
        endif ()
    endif ()

    if (IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/deps")
        add_subdirectory(deps)
    endif ()

    set_property(
            DIRECTORY
            APPEND
            PROPERTY CMAKE_CONFIGURE_DEPENDS
            "${CMAKE_CURRENT_SOURCE_DIR}/.pkg"
    )
endif ()
