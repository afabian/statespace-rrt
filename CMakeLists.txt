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
        statespace/racer/MapRacer.cpp
        statespace/racer/MapRacerVis.cpp
        statespace/racer/MapRacer.h
        statespace/racer/StateRacer.cpp
        statespace/racer/StateRacer.h
        statespace/racer/StateRacerMath.cpp
        statespace/racer/StateRacerMath.h
        motion/Motion1DInterfaceSingle.h
        motion/Motion1DInterfaceDouble.h
        motion/Motion1DPositionVelocitySingle.cpp
        motion/Motion1DPositionVelocitySingle.h
        motion/Motion1DPositionVelocityDouble.cpp
        motion/Motion1DPositionVelocityDouble.h
        motion/Motion1DPositionVelocityAccelSingle.cpp
        motion/Motion1DPositionVelocityAccelSingle.h
        motion/Motion1DPositionVelocityAccelDouble.cpp
        motion/Motion1DPositionVelocityAccelDouble.h
        motion/Motion1DPositionVelocitySingleTimed.cpp
        motion/Motion1DPositionVelocitySingleTimed.h
        motion/Motion1DPositionVelocityDoubleTimed.cpp
        motion/Motion1DPositionVelocityDoubleTimed.h
        motion/Motion1DPositionVelocityAccelSingleTimed.cpp
        motion/Motion1DPositionVelocityAccelSingleTimed.h
        motion/Motion1DPositionVelocityAccelDoubleTimed.cpp
        motion/Motion1DPositionVelocityAccelDoubleTimed.h
        statespace/racer/ModelRacer.cpp
        statespace/racer/ModelRacer.h
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

add_executable(main_modelracertest
        main_modelracertest.cpp
        statespace/racer/ModelRacer.cpp
        statespace/racer/ModelRacer.h
        statespace/racer/StateRacer.cpp
        statespace/racer/StateRacer.h
        )

target_link_libraries(main_rrtgraphtest ${PNG_LIBRARY})
