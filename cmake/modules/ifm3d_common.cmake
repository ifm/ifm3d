function(ifm3d_create_module name)
    string(TOUPPER ${name} name_uppercase)
    project(IFM3D_${name_uppercase} CXX)
    set(IFM3D_${name_uppercase}_MODULE_NAME "${name}" PARENT_SCOPE)
    set(IFM3D_${name_uppercase}_INCLUDE_DIR "${IFM3D_${name_uppercase}_SOURCE_DIR}/include" PARENT_SCOPE)
endfunction()

function(IFM3D_LINK_MODULES target) 
    set(FUNC_OPTIONS "")
    set(FUNC_SINGLE_VALUE_ARGS "")
    set(FUNC_MULTI_VALUE_ARGS "MODULES")

    cmake_parse_arguments(PARSE_ARGV 1 ARG "${FUNC_OPTIONS}" "${FUNC_SINGLE_VALUE_ARGS}" "${FUNC_MULTI_VALUE_ARGS}")

    foreach(module ${ARG_MODULES})
        if(NOT TARGET ifm3d_${module})
            message(FATAL_ERROR "Required module 'ifm3d_${module}' not found for target '${target}'")
        endif()

        target_link_libraries(${target} INTERFACE ifm3d_${module})
    endforeach(module ${ARG_MODULES})
endfunction()

function(IFM3D_MODULE name)
    set(FUNC_OPTIONS "")
    set(FUNC_SINGLE_VALUE_ARGS "")
    set(FUNC_MULTI_VALUE_ARGS "INCLUDE_DIRECTORIES;LINK_LIBRARIES;REQUIRED_MODULES")

    cmake_parse_arguments(PARSE_ARGV 1 ARG "${FUNC_OPTIONS}" "${FUNC_SINGLE_VALUE_ARGS}" "${FUNC_MULTI_VALUE_ARGS}")

    string(TOUPPER ${name} name_uppercase)
    ifm3d_create_module(${name})

    ################################################
    ## Target
    ################################################
    file(GLOB_RECURSE IFM3D_${name_uppercase}_SOURCES CONFIGURE_DEPENDS src/*.cpp src/*.h, src/*.hpp)
    file(GLOB_RECURSE IFM3D_${name_uppercase}_INCLUDES CONFIGURE_DEPENDS include/*.h include/*.hpp src/*.h, src/*.hpp)
    
    add_library(ifm3d_${name} INTERFACE)
    target_sources(ifm3d_${name} INTERFACE ${IFM3D_${name_uppercase}_SOURCES})

    if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.23")
        target_sources(ifm3d_${name} INTERFACE FILE_SET HEADERS FILES ${IFM3D_${name_uppercase}_INCLUDES})
    endif()

    #------------------
    # Compiler settings
    #------------------
    target_include_directories(ifm3d_${name}
        INTERFACE
            $<INSTALL_INTERFACE:${IFM3D_${name_uppercase}_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${IFM3D_${name_uppercase}_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${IFM3D_${name_uppercase}_BINARY_DIR}/include>
            $<BUILD_INTERFACE:${IFM3D_${name_uppercase}_SOURCE_DIR}/src> 
    )

    target_link_libraries(
        ifm3d_${name}
        INTERFACE
            ${ARG_LINK_LIBRARIES}
    )

    ifm3d_link_modules("ifm3d_${name}" MODULES ${ARG_REQUIRED_MODULES})

    ################################################
    ## Process child CMakeLists.txt files
    ################################################

    if(BUILD_TESTS)
        if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
            add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
        endif()
    else()
        message(STATUS "Skipping ${IFM3D_${name_uppercase}_MODULE_NAME} unit tests!")
    endif()

    ################################################
    ## Manage installation process
    ################################################
    if(BUILD_SDK_PKG)
        file(MAKE_DIRECTORY "${IFM3D_${name_uppercase}_BINARY_DIR}/include/")
        install(
            DIRECTORY 
                ${IFM3D_${name_uppercase}_SOURCE_DIR}/include/ 
                ${IFM3D_${name_uppercase}_BINARY_DIR}/include/
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
            COMPONENT ifm3d
            PATTERN "*.in" EXCLUDE
            PATTERN ".git" EXCLUDE
            PATTERN "*~" EXCLUDE
        )
    endif()
endfunction()

function(IFM3D_DISABLE_SOURCES module)
    set(FUNC_OPTIONS "GLOB;GLOB_RECURSE")
    set(FUNC_SINGLE_VALUE_ARGS "")
    set(FUNC_MULTI_VALUE_ARGS "")

    cmake_parse_arguments(PARSE_ARGV 1 ARG "${FUNC_OPTIONS}" "${FUNC_SINGLE_VALUE_ARGS}" "${FUNC_MULTI_VALUE_ARGS}")

    if(ARG_GLOB AND ARG_GLOB_RECURSE)
        message(FATAL_ERROR "Cannot use both GLOB and GLOB_RECURSE at the same time!")
    endif()

    string(TOUPPER ${module} module_uppercase)
    get_target_property(SOURCE_DIR ifm3d_${module} SOURCE_DIR)
    if(NOT SOURCE_DIR)
        message(FATAL_ERROR "Target ifm3d_${module} not found or has no SOURCE_DIR property")
    endif()
    get_target_property(target_type ifm3d_${module} TYPE)

    set(files "")
    foreach(file ${ARG_UNPARSED_ARGUMENTS})
        if(IS_ABSOLUTE ${file})
            list(APPEND files ${file})
        else()
            list(APPEND files "${SOURCE_DIR}/${file}")
        endif()
    endforeach()

    if(ARG_GLOB)
        file(GLOB remove_sources CONFIGURE_DEPENDS ${files})
    elseif(ARG_GLOB_RECURSE)
        file(GLOB_RECURSE remove_sources CONFIGURE_DEPENDS ${files})
    else()
        set(remove_sources ${files})
    endif()

    if(target_type STREQUAL "INTERFACE_LIBRARY")
        get_target_property(iface_srcs ifm3d_${module} INTERFACE_SOURCES)
        if(iface_srcs)
            foreach(src ${remove_sources})
                list(REMOVE_ITEM iface_srcs ${src})
            endforeach()
            set_property(TARGET ifm3d_${module} PROPERTY INTERFACE_SOURCES "${iface_srcs}")
        endif()
    else()
        foreach(source ${remove_sources})
            if(EXISTS "${source}")
                set_source_files_properties(${source} PROPERTIES HEADER_FILE_ONLY TRUE)
            endif()
        endforeach()
    endif()
endfunction()