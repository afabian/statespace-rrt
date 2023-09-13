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
    Map2D map_2d("maps/2d/walls.png");
    State2DMath state_math_2d(1);
    RRT<State2D,State2DMath,Map2D> rrt_2d(&map_2d, &state_math_2d);
    State2D start_2d{10*5, 215*5};
    State2D goal_2d(275*5, 15*5);
    rrt_2d.setStartState(&start_2d);
    rrt_2d.setGoalState(&goal_2d, 0.01);
    rrt_2d.configureSampling(5001, false);
    rrt_2d.configureRewiring(true, 0.05, 10);
    rrt_2d.configureDebugOutput(true, true, "output/2d/walls/");
    rrt_2d.run();
    cout << "2d Walls: Final path cost: " << rrt_2d.getGoalCost() << endl;
}

void main_2d_field() {
    Map2D map_2d("maps/2d/field.png");
    State2DMath state_math_2d(10);
    RRT<State2D,State2DMath,Map2D> rrt_2d(&map_2d, &state_math_2d);
    State2D start_2d{50, 950};
    State2D goal_2d(950, 50);
    rrt_2d.setStartState(&start_2d);
    rrt_2d.setGoalState(&goal_2d, 0.01);
    rrt_2d.configureSampling(5001, false);
    rrt_2d.configureRewiring(true, 0.05, 10);
    rrt_2d.configureDebugOutput(true, true, "output/2d/field/");
    rrt_2d.run();
    cout << "2D Field: Final path cost: " << rrt_2d.getGoalCost() << endl;
}

void main_2d_elevation() {
    Map2D map_2d("maps/2d/elevation.png");
    State2DElevationMath state_math_2d(1000);
    RRT<State2D,State2DElevationMath,Map2D> rrt_2d(&map_2d, &state_math_2d);
    State2D start_2d{50, 50};
    State2D goal_2d(550, 550);
    rrt_2d.setStartState(&start_2d);
    rrt_2d.setGoalState(&goal_2d, 0.01);
    rrt_2d.configureSampling(10001, false);
    rrt_2d.configureRewiring(true, 0.05, 10);
    rrt_2d.configureDebugOutput(true, true, "output/2d/elevation/");
    rrt_2d.run();
    cout << "2D Elevation: Final path cost: " << rrt_2d.getGoalCost() << endl;
}

void main_3d() {
    Map3D map_3d("maps/3d/test1.txt");
    State3DMath state_math_3d(1);
    RRT<State3D,State3DMath,Map3D> rrt_3d(&map_3d, &state_math_3d);
    State3D start_3d{5, 5, 5};
    State3D goal_3d(95, 95, 95);
    rrt_3d.setStartState(&start_3d);
    rrt_3d.setGoalState(&goal_3d, 0.05);
    rrt_3d.configureSampling(2001, true);
    rrt_3d.configureRewiring(true, 0.25, 10);
    rrt_3d.configureDebugOutput(true, true, "output/3d/");
    rrt_3d.run();
    cout << "3D: Final path cost: " << rrt_3d.getGoalCost() << endl;
}

void main_floater() {
    // RRT<StateFloater,StateFloaterMath,MapFloater> rrt_floater;
    // StateFloater start_floater{10, 20, 30};
    // StateFloater goal_floater(100, 100, 100);
    // MapFloater map_float('maps/floater/test1.txt');

    // rrt_floater.setStartState(start_floater);
    // rrt_floater.setGoalState(goal_floater);
    // rrt_floater.setMap(&map_floater);
    // rrt_floater.runOnce();
}

int main(int argc, char* argv[]) {
    auto start = high_resolution_clock::now();

    if (argc == 1 || strcmp(argv[1], "2d_walls") == 0) {
        main_2d_walls();
    }
    if (argc == 1 || strcmp(argv[1], "2d_field") == 0) {
        main_2d_field();
    }
    if (argc == 1 || strcmp(argv[1], "2d_elevation") == 0) {
        main_2d_elevation();
    }
    if (argc == 1 || strcmp(argv[1], "3d") == 0) {
        main_3d();
    }
    if (argc == 1 || strcmp(argv[1], "floater") == 0) {
        main_floater();
    }

    auto finish = high_resolution_clock::now();
    auto elapsed = duration_cast<milliseconds>(finish - start);
    cout << "Elapsed time: " << ((double)elapsed.count()/1000) << endl;

    return 0;
}