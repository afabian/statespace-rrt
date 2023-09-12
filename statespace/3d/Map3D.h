#ifndef RRT_MAP3D_H
#define RRT_MAP3D_H

#include "State3D.h"
#include <string>
#include <png.h>

#define MAP3D_MAX_OBJECTS 1000

class Map3DObject {
public:
    State3D bound_lower;
    State3D bound_upper;
};

class Map3D {

friend class State3DMath;

public:
    Map3D(std::string datafile);

    void getBounds(State3D* minimums, State3D* maximums);

    bool pointInObstacle(State3D* point);
    bool edgeInObstacle(State3D* pointA, State3D* pointB);
    float pointCost(State3D* point);
    float edgeCost(State3D* pointA, State3D* pointB);

    void resetVis();
    void addVisPoint(State3D* point, int color);
    void addVisLine(State3D* pointA, State3D* pointB, int color);
    void renderVis(std::string filename_prefix);
    void renderFinalVis(std::string filename_prefix);

protected:
    Map3DObject border;
    Map3DObject objects[MAP3D_MAX_OBJECTS];
    int object_count = 0;

    std::string html = "";

};


#endif
