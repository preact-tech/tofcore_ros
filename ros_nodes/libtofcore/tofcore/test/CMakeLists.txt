find_package(GTest CONFIG
    COMPONENTS
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        GTest::gmock_main
)



add_subdirectory (functional-tests)
