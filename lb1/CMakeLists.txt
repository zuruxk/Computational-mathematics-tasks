cmake_minimum_required(VERSION 3.10)
project(Lab1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

add_executable(Lab1 
    main.cpp
    modules/dif/dif.cpp 
    modules/render/render.cpp
)

target_include_directories(Lab1 PRIVATE ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
target_link_libraries(Lab1 PRIVATE ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} m)