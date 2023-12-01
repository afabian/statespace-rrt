#include "RRT.h"

#include "statespace/2d/State2D.h"
#include "statespace/2d/State2DMath.h"
#include "statespace/2d/State2DElevationMath.h"
#include "statespace/2d/Map2D.h"

#include "statespace/3d/State3D.h"
#include "statespace/3d/State3DMath.h"
#include "statespace/3d/Map3D.h"

 #include "statespace/floater/StateFloater.h"
 #include "statespace/floater/StateFloaterMath.h"
 #include "statespace/floater/MapFloater.h"

#include <iostream>
#include <cstring>
#include <chrono>

using namespace std;
using namespace std::chrono;

void main_2d_walls() {
    Map2D map("maps/2d/walls.png");
    State2DMath state_math(1);
    RRT<State2D,State2DMath,Map2D> rrt(&map, &state_math);
    State2D start{10*5, 215*5};
    State2D goal(275*5, 15*5);
    rrt.setStartState(&start);
    rrt.setGoalState(&goal, 0.01);
    rrt.configureSampling(5001, false);
    rrt.configureRewiring(true, 0.05, 10);
    rrt.configureDebugOutput(true, true, "output/2d/walls/", 0, 0);
    rrt.run();
    cout << "2d Walls: Final path cost: " << rrt.getGoalCost() << endl;
}

void main_2d_field() {
    Map2D map("maps/2d/field.png");
    State2DMath state_math(10);
    RRT<State2D,State2DMath,Map2D> rrt(&map, &state_math);
    State2D start{50, 950};
    State2D goal(950, 50);
    rrt.setStartState(&start);
    rrt.setGoalState(&goal, 0.01);
    rrt.configureSampling(5001, false);
    rrt.configureRewiring(true, 0.05, 10);
    rrt.configureDebugOutput(true, true, "output/2d/field/", 0, 0);
    rrt.run();
    cout << "2D Field: Final path cost: " << rrt.getGoalCost() << endl;
}

void main_2d_elevation() {
    Map2D map("maps/2d/elevation.png");
    State2DElevationMath state_math(1000);
    RRT<State2D,State2DElevationMath,Map2D> rrt(&map, &state_math);
    State2D start{50, 50};
    State2D goal(550, 550);
    rrt.setStartState(&start);
    rrt.setGoalState(&goal, 0.01);
    rrt.configureSampling(10001, false);
    rrt.configureRewiring(true, 0.05, 10);
    rrt.configureDebugOutput(true, true, "output/2d/elevation/", 0, 0);
    rrt.run();
    cout << "2D Elevation: Final path cost: " << rrt.getGoalCost() << endl;
}

void main_3d() {
    Map3D map_3d("maps/3d/test1.txt");
    State3DMath state_math_3d(1);
    RRT<State3D,State3DMath,Map3D> rrt(&map_3d, &state_math_3d);
    State3D start_3d{5, 5, 5};
    State3D goal_3d(95, 95, 95);
    rrt.setStartState(&start_3d);
    rrt.setGoalState(&goal_3d, 0.05);
    rrt.configureSampling(1001, true);
    rrt.configureRewiring(true, 0.25, 10);
    rrt.configureDebugOutput(true, true, "output/3d/", 1920, 1080);
    rrt.run();
    cout << "3D: Final path cost: " << rrt.getGoalCost() << endl;
}

void main_floater() {
    MapFloater map("maps/floater/floater.png", 100);
    StateFloaterMath state_math(20, 3);
    RRT<StateFloater,StateFloaterMath,MapFloater> rrt(&map, &state_math);
    StateFloater start{50, 550, 0};
    StateFloater goal(950, 50, 0);
    rrt.setStartState(&start);
    rrt.setGoalState(&goal, 0.1);
    rrt.configureSampling(5001, true);
    rrt.configureRewiring(true, 0.05, 10);
    rrt.configureDebugOutput(true, true, "output/floater/", 0, 0);
    rrt.run();
    cout << "Floater: Final path cost: " << rrt.getGoalCost() << endl;
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