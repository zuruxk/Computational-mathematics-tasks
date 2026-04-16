cmake_minimum_required(VERSION 3.10)

project(DotProduct VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(dot_product
    main.cpp
    modules/ExactDouble/ExactDouble.cpp
    modules/DotProduct/DotProduct.cpp
)

target_include_directories(dot_product PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(tests
    tests.cpp
    modules/ExactDouble/ExactDouble.cpp
    modules/DotProduct/DotProduct.cpp
)

target_include_directories(tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

find_package(GTest REQUIRED)
target_link_libraries(tests GTest::gtest_main)