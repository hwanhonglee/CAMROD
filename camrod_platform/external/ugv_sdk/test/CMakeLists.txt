# Use the host/toolchain-provided GoogleTest package.  The SDK is vendored as
# ordinary source in CAMROD, so its tests must not assume a nested googletest
# submodule and must not download dependencies during a normal build.
message(STATUS "Build unit tests with system GoogleTest.")
find_package(GTest CONFIG REQUIRED)

# add unit tests
add_executable(utests 
    unit_tests/utest_scout_v1_command.cpp
    unit_tests/utest_hunter_v1_command.cpp)
target_link_libraries(utests PRIVATE GTest::gtest_main ${PROJECT_NAME})
get_target_property(PRIVATE_HEADERS ${PROJECT_NAME} INCLUDE_DIRECTORIES)
target_include_directories(utests PRIVATE ${PRIVATE_HEADERS})

# add_subdirectory(devel)
add_executable(scout_memory_check scout_memory_check.cpp)
target_link_libraries(scout_memory_check ugv_sdk)
