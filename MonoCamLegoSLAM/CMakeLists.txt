cmake_minimum_required(VERSION 3.1)

set(header_files
    include/Camera.hpp
    include/Image.hpp
    include/KeyLogger.hpp
    include/LineSegmentDetector.hpp
    include/VanishingPointEstimator.hpp
)

set(source_files
    src/Camera.cpp
    src/Image.cpp
    src/KeyLogger.cpp
    src/LineSegmentDetector.cpp
    src/VanishingPointEstimator.cpp
)

add_library(monocamLegoSLAM STATIC ${header_files} ${source_files})

target_include_directories(monocamLegoSLAM PUBLIC include)

target_link_libraries(monocamLegoSLAM
    PRIVATE lsd
)

