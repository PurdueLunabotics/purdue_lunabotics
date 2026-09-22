cmake_minimum_required(VERSION 3.10)

project(teensyduino VERSION 1.0 LANGUAGES C CXX)

file(GLOB_RECURSE teensy4_core_src CONFIGURE_DEPENDS
    "${teensy_package_SOURCE_DIR}/cores/teensy4/*.c"
    "${teensy_package_SOURCE_DIR}/cores/teensy4/*.cpp"
)
list(REMOVE_ITEM teensy4_core_src "${teensy_package_SOURCE_DIR}/cores/teensy4/main.cpp")

add_library(teensy4_core STATIC ${teensy4_core_src})
target_include_directories(teensy4_core PUBLIC ${teensy_package_SOURCE_DIR}/cores/teensy4)
add_library(teensyduino::teensy4_core ALIAS teensy4_core)

function(create_arduino_lib DIRECTORY LIBRARIES_DIRS)
    cmake_path(GET DIRECTORY FILENAME TARGET_NAME)
    file(GLOB srcs CONFIGURE_DEPENDS
        "${DIRECTORY}/*.c"
        "${DIRECTORY}/*.cpp"
        "${DIRECTORY}/src/*.c"
        "${DIRECTORY}/src/*.cpp"
    )
    if (NOT srcs)
        message(WARNING "${TARGET_NAME} is an empty library and will not be included")
        return()
    endif()
    add_library(${TARGET_NAME} EXCLUDE_FROM_ALL STATIC ${srcs})
    if(IS_DIRECTORY "${DIRECTORY}/src")
        target_include_directories(${TARGET_NAME} PUBLIC ${DIRECTORY}/src)
    else()
        target_include_directories(${TARGET_NAME} PUBLIC ${DIRECTORY})
    endif()
    target_link_libraries(${TARGET_NAME} PUBLIC teensy4_core)
    execute_process(
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/find_deps.py ${DIRECTORY} "${LIBRARIES_DIRS}"
        OUTPUT_VARIABLE deps
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    target_link_libraries(${TARGET_NAME} PUBLIC ${deps})
    add_library(teensyduino::${TARGET_NAME} ALIAS ${TARGET_NAME})
endfunction()

file(GLOB arduino_libs_dirs "${teensy_package_SOURCE_DIR}/libraries/*")
foreach(arduino_lib ${arduino_libs_dirs})
    if(IS_DIRECTORY "${arduino_lib}")
        create_arduino_lib(${arduino_lib} "${arduino_libs_dirs}")
    endif()
endforeach()

