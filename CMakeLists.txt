cmake_minimum_required(VERSION 3.26)
project(rrt)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})

add_executable(main
        RRTGraph.cpp
        RRTGraph.h
        Graph.cpp
        Graph.h
        main.cpp
        RRT.cpp
        RRT.h
        State2D.cpp
        State2D.h
        State2DMath.cpp
        State2DMath.h
        State3D.cpp
        State3D.h
        State3DMath.cpp
        State3DMath.h
        StateFloater.cpp
        StateFloater.h
        StateFloaterMath.cpp
        StateFloaterMath.h
        Map2D.cpp
        Map2D.h
        Map3D.cpp
        Map3D.h)

target_link_libraries(main ${PNG_LIBRARY})

add_executable(main_rrtgraphtest
        RRTGraph.cpp
        RRTGraph.h
        main_rrtgraphtest.cpp
        State2D.cpp
        State2D.h
        State2DMath.cpp
        State2DMath.h)

add_executable(main_graphtest
        Graph.cpp
        Graph.h
        main_graphtest.cpp
        State2D.cpp
        State2D.h
        State2DMath.cpp
        State2DMath.h)
