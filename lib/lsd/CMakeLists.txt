cmake_minimum_required(VERSION 3.1)

set(header_files
    include/lsd.hpp
)

set(source_files
    src/lsd.cpp
)

add_library(lsd STATIC ${header_files} ${source_files})

target_include_directories(lsd PUBLIC include)
