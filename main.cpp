#include "RRT.h"
#include "statespace/2d/State2D.h"
#include "statespace/2d/State2DMath.h"
#include "statespace/2d/State2DElevationMath.h"
#include "statespace/2d/Map2D.h"
#include "statespace/3d/State3D.h"
#include "statespace/3d/State3DMath.h"
#include "statespace/3d/Map3D.h"
// #include "StateFloater.h"
// #include "StateFloaterMath.h"
// #include "MapFloater.h"

#include <iostream>
#include <cstring>
#include <chrono>

using namespace std;
using namespace std::chrono;

void main_2d_walls() {
    RRT<State2D,State2DMath,Map2D> rrt_2d;
    Map2D map_2d("d:/statespace-rrt/maps/2d/walls.png");
    rrt_2d.setMap(&map_2d);
    State2DMath state_math_2d(1);
    rrt_2d.setStateMath(&state_math_2d);
    State2D start_2d{10*5, 215*5};
    State2D goal_2d(275*5, 15*5);
    rrt_2d.setStartState(&start_2d);
    rrt_2d.setGoalState(&goal_2d);
    rrt_2d.configureSampling(5001, false);
    rrt_2d.configureRewiring(true, 0.05, 10);
    rrt_2d.configureDebugOutput(true, true, "d:/statespace-rrt/output/2d/walls/");
    rrt_2d.run();
    cout << "Final 2D path cost: " << rrt_2d.getGoalCost() << endl;
}

void main_2d_field() {
    RRT<State2D,State2DMath,Map2D> rrt_2d;
    Map2D map_2d("d:/statespace-rrt/maps/2d/field.png");
    rrt_2d.setMap(&map_2d);
    State2DMath state_math_2d(10);
    rrt_2d.setStateMath(&state_math_2d);
    State2D start_2d{50, 950};
    State2D goal_2d(950, 50);
    rrt_2d.setStartState(&start_2d);
    rrt_2d.setGoalState(&goal_2d);
    rrt_2d.configureSampling(5001, false);
    rrt_2d.configureRewiring(true, 0.05, 10);
    rrt_2d.configureDebugOutput(true, true, "d:/statespace-rrt/output/2d/field/");
    rrt_2d.run();
    cout << "Final 2D path cost: " << rrt_2d.getGoalCost() << endl;
}

void main_2d_elevation() {
    RRT<State2D,State2DElevationMath,Map2D> rrt_2d;
    Map2D map_2d("d:/statespace-rrt/maps/2d/elevation.png");
    rrt_2d.setMap(&map_2d);
    State2DElevationMath state_math_2d(1000);
    rrt_2d.setStateMath(&state_math_2d);
    State2D start_2d{50, 50};
    State2D goal_2d(550, 550);
    rrt_2d.setStartState(&start_2d);
    rrt_2d.setGoalState(&goal_2d);
    rrt_2d.configureSampling(10001, false);
    rrt_2d.configureRewiring(true, 0.05, 10);
    rrt_2d.configureDebugOutput(true, true, "d:/statespace-rrt/output/2d/elevation/");
    rrt_2d.run();
    cout << "Final 2D path cost: " << rrt_2d.getGoalCost() << endl;
}

void main_3d() {
    RRT<State3D,State3DMath,Map3D> rrt_3d;
    Map3D map_3d("d:/statespace-rrt/maps/3d/test1.txt");
    rrt_3d.setMap(&map_3d);
    State3DMath state_math_3d(1);
    rrt_3d.setStateMath(&state_math_3d);
    State3D start_3d{5, 5, 5};
    State3D goal_3d(95, 95, 95);
    rrt_3d.setStartState(&start_3d);
    rrt_3d.setGoalState(&goal_3d);
    rrt_3d.configureSampling(1001, false);
    rrt_3d.configureRewiring(false, 0.05, 10);
    rrt_3d.configureDebugOutput(true, true, "d:/statespace-rrt/output/3d/");
    rrt_3d.run();
    cout << "Final 3D path cost: " << rrt_3d.getGoalCost() << endl;
}

void main_floater() {
    // RRT<StateFloater,StateFloaterMath,MapFloater> rrt_floater;
    // StateFloater start_floater{10, 20, 30};
    // StateFloater goal_floater(100, 100, 100);
    // MapFloater map_float('d:/statespace-rrt/maps/floater/test1.txt');

    // rrt_floater.setStartState(start_floater);
    // rrt_floater.setGoalState(goal_floater);
    // rrt_floater.setMap(&map_floater);
    // rrt_floater.runOnce();
}

int main(int argc, char* argv[]) {
    auto start = high_resolution_clock::now();

    if (strcmp(argv[1], "2d_walls") == 0) {
        main_2d_walls();
    }
    if (strcmp(argv[1], "2d_field") == 0) {
        main_2d_field();
    }
    if (strcmp(argv[1], "2d_elevation") == 0) {
        main_2d_elevation();
    }
    if (strcmp(argv[1], "3d") == 0) {
        main_3d();
    }
    if (strcmp(argv[1], "floater") == 0) {
        main_floater();
    }

    auto finish = high_resolution_clock::now();
    auto elapsed = duration_cast<milliseconds>(finish - start);
    cout << "Elapsed time: " << ((double)elapsed.count()/1000) << endl;

    return 0;
}