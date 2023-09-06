#ifndef STATE3DMATH_H
#define STATE3DMATH_H

#include "State3D.h"
#include "Map3D.h"

class State3DMath {
  
public:
    State3DMath();
    State3DMath(float scale);

    void setMap(Map3D* _map);
    void setCostScale(float scale);

    bool pointInObstacle(State3D* point);
    bool edgeInObstacle(State3D* pointA, State3D* pointB);

    float pointCost(State3D* point);
    float edgeCost(State3D* pointA, State3D* pointB);

    double distance(State3D* a, State3D* b);
    double approx_distance(State3D* a, State3D* b);

    void setRandomStateConstraints(State3D _minimums, State3D _maximums);
    State3D getRandomState();

protected:
    State3D minimums, maximums;
    State3D scale, shift;

    float cost_scale = 1;

    const float EDGE_WALK_SCALE = 1.0f;

    Map3D* map = nullptr;
};

#endif