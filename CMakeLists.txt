cmake_minimum_required(VERSION 3.22)
project(hyperion VERSION 23.05.09 DESCRIPTION "" HOMEPAGE_URL https://github.com/VIS4ROB-lab/hyperion LANGUAGES CXX)

option(BUILD_TESTING "Builds the tests" ON)
option(BUILD_EXAMPLES "Builds the examples" ON)
option(BUILD_BENCHMARKS "Builds the benchmarks" OFF)

if (PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED YES)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -march=native -Wall -Wextra -Wnon-virtual-dtor -Wno-unused-function -pedantic")
endif ()

find_package(ament_cmake REQUIRED)

find_package(absl REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

file(GLOB_RECURSE PROJECT_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/*.hpp ${PROJECT_SOURCE_DIR}/include/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PROJECT_FILES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        "/usr/local/include/sym")

set(PROJECT_LIBRARIES
        absl::base
        absl::flat_hash_map
        absl::strings
        Ceres::ceres
        Eigen3::Eigen
        glog::glog
        symforce_gen
        symforce_opt
        yaml-cpp
        fmt::fmt
)

set(PROJECT_DEPENDENCIES
        absl
        Ceres
        Eigen3
        glog
        nav_msgs
        rclcpp
        std_msgs
        tf2_ros
        visualization_msgs
        fmt
)

target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})
ament_export_dependencies(${PROJECT_DEPENDENCIES})

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}_export
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

MACRO(SUBDIRLIST result dir)
    FILE(GLOB children RELATIVE ${dir} ${dir}/*)
    SET(dirlist "")
    FOREACH (child ${children})
        IF (IS_DIRECTORY ${dir}/${child})
            LIST(APPEND dirlist ${dir}/${child})
        ENDIF ()
    ENDFOREACH ()
    SET(${result} ${dirlist})
ENDMACRO()

if (BUILD_EXAMPLES)
    SUBDIRLIST(SUBDIRS ${PROJECT_SOURCE_DIR}/apps)
    FOREACH (subdir ${SUBDIRS})
        add_subdirectory(${subdir})
    ENDFOREACH ()
endif ()

if (BUILD_TESTING)
    add_subdirectory(tests)
    #find_package(ament_lint_auto REQUIRED)
    #ament_lint_auto_find_test_dependencies()
endif ()

if (BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif ()

ament_package()
