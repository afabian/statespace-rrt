#ifndef RRT_MAP3D_H
#define RRT_MAP3D_H

#include "State3D.h"
#include <string>
#include <png.h>

class Map3D {

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
    void renderVis(std::string pngfile);

private:

};


#endif
