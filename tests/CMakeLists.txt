project(hyperion_tests)

find_package(Catch2 REQUIRED)
find_package(ament_cmake_catch2 REQUIRED)
file(GLOB_RECURSE PROJECT_FILES CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/*.hpp ${PROJECT_SOURCE_DIR}/src/*.cpp)

if (PROJECT_FILES)
    ament_add_catch2(${PROJECT_NAME} ${PROJECT_FILES})
    target_include_directories(${PROJECT_NAME} PRIVATE
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${PROJECT_NAME} PRIVATE hyperion::hyperion PRIVATE Catch2::Catch2WithMain)
endif ()
