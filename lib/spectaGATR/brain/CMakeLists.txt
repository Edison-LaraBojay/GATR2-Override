# spectaGATR Brain publisher: portable core and host tests. pros/ holds the
# PROS-only adapter; PROS projects build it through brain/gatr2_brain.mk.

find_package(Threads REQUIRED)

add_library(spectagatr_publisher STATIC
    src/publisher.cpp
)
target_include_directories(spectagatr_publisher PUBLIC include)
target_link_libraries(spectagatr_publisher PUBLIC spectagatr_protocol)
target_compile_options(spectagatr_publisher PRIVATE ${SPECTAGATR_WARNINGS})

if(SPECTAGATR_BUILD_TESTS)
    # Test files.
    set(PUBLISHER_TEST_SOURCES
        tests/pose_gtest.cpp
        tests/publisher_gtest.cpp
        tests/aux_gtest.cpp
        tests/concurrency_gtest.cpp
    )
    add_executable(spectagatr_publisher_tests ${PUBLISHER_TEST_SOURCES})
    target_link_libraries(spectagatr_publisher_tests
        PRIVATE spectagatr_publisher Threads::Threads GTest::gtest_main)
    target_compile_options(spectagatr_publisher_tests PRIVATE ${SPECTAGATR_WARNINGS})
    gtest_discover_tests(spectagatr_publisher_tests)

    # Own executable: it replaces the global operator new.
    add_executable(spectagatr_publisher_allocation_tests tests/allocation_gtest.cpp)
    target_link_libraries(spectagatr_publisher_allocation_tests
        PRIVATE spectagatr_publisher GTest::gtest_main)
    target_compile_options(spectagatr_publisher_allocation_tests PRIVATE ${SPECTAGATR_WARNINGS})
    gtest_discover_tests(spectagatr_publisher_allocation_tests)
endif()
