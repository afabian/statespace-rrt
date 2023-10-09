cmake_minimum_required(VERSION 3.26)
project(rrt)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})

add_executable(main
        RRTGraph.cpp
        RRTGraph.h
        main.cpp
        RRT.cpp
        RRT_Rewire.cpp
        RRT_Sampling.cpp
        RRT_Output.cpp
        RRT.h
        Utils.cpp
        Utils.h
        statespace/2d/State2D.cpp
        statespace/2d/State2D.h
        statespace/2d/State2DMath.cpp
        statespace/2d/State2DMath.h
        statespace/2d/State2DElevationMath.cpp
        statespace/2d/State2DElevationMath.h
        statespace/2d/Map2DVis.cpp
        statespace/2d/Map2D.cpp
        statespace/2d/Map2D.h
        statespace/3d/State3D.cpp
        statespace/3d/State3D.h
        statespace/3d/State3DMath.cpp
        statespace/3d/State3DMath.h
        statespace/3d/Map3DVis.cpp
        statespace/3d/Map3D.cpp
        statespace/3d/Map3D.h
        statespace/floater/StateFloater.cpp
        statespace/floater/StateFloater.h
        statespace/floater/StateFloaterMath.cpp
        statespace/floater/StateFloaterMath.h
        statespace/floater/MapFloater.cpp
        statespace/floater/MapFloater.h
        statespace/floater/MapFloaterVis.cpp
)

target_link_libraries(main ${PNG_LIBRARY})

add_executable(main_rrtgraphtest
        RRTGraph.cpp
        RRTGraph.h
        main_rrtgraphtest.cpp
        statespace/2d/Map2DVis.cpp
        statespace/2d/Map2D.cpp
        statespace/2d/Map2D.h
        statespace/2d/State2D.cpp
        statespace/2d/State2D.h
        statespace/2d/State2DMath.cpp
        statespace/2d/State2DMath.h)

target_link_libraries(main_rrtgraphtest ${PNG_LIBRARY})
