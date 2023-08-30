#include "RRT.h"
#include "statespace/2d/State2D.h"
#include "statespace/2d/State2DMath.h"
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

void main_2d() {
    RRT<State2D,State2DMath,Map2D> rrt_2d;
    State2D start_2d{10*5, 215*5};
    State2D goal_2d(275*5, 15*5);
    Map2D map_2d("d:/statespace-rrt/maps/2d/test1.png");
    rrt_2d.setStartState(&start_2d);
    rrt_2d.setGoalState(&goal_2d);
    rrt_2d.setMap(&map_2d);
    rrt_2d.configureSampling(5001, false);
    rrt_2d.configureRewiring(true, 0.05, 10);
    rrt_2d.configureDebugOutput(true, true, "d:/statespace-rrt/output/2d/");
    rrt_2d.run();
    cout << "Final 2D path cost: " << rrt_2d.getGoalCost() << endl;
}

void main_3d() {
    RRT<State3D,State3DMath,Map3D> rrt_3d;
    State3D start_3d{5, 5, 5};
    State3D goal_3d(95, 95, 95);
    Map3D map_3d("d:/statespace-rrt/maps/3d/test1.txt");
    rrt_3d.setStartState(&start_3d);
    rrt_3d.setGoalState(&goal_3d);
    rrt_3d.setMap(&map_3d);
    rrt_3d.configureSampling(5001, false);
    rrt_3d.configureRewiring(true, 0.05, 10);
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

    if (strcmp(argv[1], "2d") == 0) {
        main_2d();
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