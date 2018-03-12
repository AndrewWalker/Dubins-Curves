cmake_minimum_required(VERSION 3.0)

add_subdirectory(3rd_party/google-test)

add_library(dubins 
    src/dubins.c)

target_include_directories(dubins 
    PUBLIC 
    include)

target_link_libraries(dubins 
    PUBLIC 
    m)

enable_testing()

add_executable(unittest_dubins
    tests/montecarlo_tests.cpp
    tests/stableapi_tests.cpp)

target_link_libraries(unittest_dubins
    dubins
    GTest::Main
    GTest::GTest)

add_test(unittest_dubins unittest_dubins)
