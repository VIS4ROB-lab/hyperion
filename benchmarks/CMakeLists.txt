project(hyperion_benchmarks)

include(FetchContent)
FetchContent_Declare(lie_spline_experiments GIT_REPOSITORY https://gitlab.com/tum-vision/lie-spline-experiments.git)
FetchContent_MakeAvailable(lie_spline_experiments)

find_package(Catch2 REQUIRED)
find_package(ament_cmake_catch2 REQUIRED)
file(GLOB_RECURSE PROJECT_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/*.hpp ${PROJECT_SOURCE_DIR}/src/*.cpp)

if (PROJECT_FILES)
    ament_add_catch2(${PROJECT_NAME} ${PROJECT_FILES})
    target_include_directories(${PROJECT_NAME} PRIVATE
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            ${lie_spline_experiments_SOURCE_DIR}/include
    )
    target_link_libraries(${PROJECT_NAME} PRIVATE hyperion::hyperion PRIVATE basalt::basalt-headers PRIVATE Catch2::Catch2WithMain)
endif ()
