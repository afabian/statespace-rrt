#include "RRT.h"
#include "State2D.h"
#include "State2DMath.h"
#include "Map2D.h"
#include "State3D.h"
#include "State3DMath.h"
#include "Map3D.h"
// #include "StateFloater.h"
// #include "StateFloaterMath.h"
// #include "MapFloater.h"
#include <iostream>

using namespace std;

int main(int argc, char* argv[]) {

    RRT<State2D,State2DMath,Map2D> rrt_2d;
    State2D start_2d{10*5, 215*5};
    State2D goal_2d(275*5, 15*5);
    Map2D map_2d("d:/rrt/maps/2d/test1.png");
    rrt_2d.setStartState(&start_2d);
    rrt_2d.setGoalState(&goal_2d);
    rrt_2d.setMap(&map_2d);
    for (int i=0; i<5001; i++) {
        rrt_2d.runOnce();
        if ((i < 10) || (i < 100 && i % 10 == 0) || (i < 1000 && i % 100 == 0) || (i < 10000 && i % 1000 == 0) || (i < 100000 && i % 10000 == 0)) {
            rrt_2d.renderVis();
            map_2d.renderVis("d:/rrt/output/2d/test" + to_string(i) + ".png");
        }
    }
    cout << "Final path cost: " << rrt_2d.getGoalCost() << endl;
    for (int i=0; i<10; i++) {
        rrt_2d.rewireAll();
        cout << "Final path cost: " << rrt_2d.getGoalCost() << endl;
        rrt_2d.renderVis();
        map_2d.renderVis("d:/rrt/output/2d/test_rewire_" + to_string(i) + ".png");
    }

    RRT<State3D,State3DMath,Map3D> rrt_3d;
    State3D start_3d{10, 20, 30};
    State3D goal_3d(100, 100, 100);
    Map3D map_3d("d:/rrt/maps/3d/test1.txt");

    rrt_3d.setStartState(&start_3d);
    rrt_3d.setGoalState(&goal_3d);
    rrt_3d.setMap(&map_3d);
    rrt_3d.runOnce();

    // RRT<StateFloater,StateFloaterMath,MapFloater> rrt_floater;
    // StateFloater start_floater{10, 20, 30};
    // StateFloater goal_floater(100, 100, 100);
    // MapFloater map_float('d:/rrt/maps/floater/test1.txt');

    // rrt_floater.setStartState(start_floater);
    // rrt_floater.setGoalState(goal_floater);
    // rrt_floater.setMap(&map_floater);
    // rrt_floater.runOnce();

    return 0;
}