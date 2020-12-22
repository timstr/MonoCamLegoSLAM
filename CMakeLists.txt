cmake_minimum_required(VERSION 3.1)

# set(CMAKE_CC_COMPILER arm-linux-gnueabi-gcc)
# set(CMAKE_CXX_COMPILER arm-linux-gnueabi-g++)

project(MonoCamLegoSLAM)

add_subdirectory(lib)

add_subdirectory(MonoCamLegoSLAM)

function(custom_add_ev3_executable target sources)
    set(sources_list ${sources} ${ARGN})
    add_executable(${target} ${sources_list})
    target_link_libraries(${target} PUBLIC ev3dev monocamLegoSLAM)
    target_compile_options(${target} PRIVATE -Wall -Wno-psabi -flto)
endfunction()

custom_add_ev3_executable(battery app/battery.cpp)

custom_add_ev3_executable(capture app/capture.cpp)

custom_add_ev3_executable(stop app/stop.cpp)

custom_add_ev3_executable(sensor_test app/sensor_test.cpp)

custom_add_ev3_executable(drive_test app/drive_test.cpp)

custom_add_ev3_executable(vp app/vp.cpp)
