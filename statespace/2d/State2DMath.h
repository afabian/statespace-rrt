#ifndef STATE2DMATH_H
#define STATE2DMATH_H

#include "State2D.h"
#include "Map2D.h"

class State2DMath {
  
public:
    State2DMath();
    State2DMath(float scale);

    void setMap(Map2D* _map);
    void setCostScale(float _scale);

    bool pointInObstacle(State2D* point);
    bool edgeInObstacle(State2D* pointA, State2D* pointB);

    float pointCost(State2D* point);
    float edgeCost(State2D* pointA, State2D* pointB);

    double distance(State2D* source, State2D* dest);
    double approx_distance(State2D* source, State2D* dest);

    void setRandomStateConstraints(State2D _minimums, State2D _maximums);
    State2D getRandomState();

protected:
    State2D minimums, maximums;
    State2D scale, shift;

    float cost_scale = 1;

    const float EDGE_WALK_SCALE = 1.0f;

    Map2D* map = nullptr;
};

#endif