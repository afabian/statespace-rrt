#ifndef STATEFLOATERMATH_H
#define STATEFLOATERMATH_H

#include "StateFloater.h"
#include "MapFloater.h"

// "Floater" is a sample system, with a hypothetical vehicle that moves forward
// at a constant rate (so we could say it's position x equals time t), but
// is free to move vertically.  In the vertical (y) axis, it has gravity and inertia,
// and can command update thrust between 0% and 100% of some maximum value.
//
// Maps are developed with obstacles in x-y space, so that the vehicle must
// time its thrust commands to avoid the obstacle.

class StateFloaterMath {
  
public:
    StateFloaterMath();

    void setMap(MapFloater* _map);

    bool pointInObstacle(StateFloater* point);
    bool edgeInObstacle(StateFloater* pointA, StateFloater* pointB);

    float pointCost(StateFloater* point);
    float edgeCost(StateFloater* pointA, StateFloater* pointB);

    double distance(StateFloater* a, StateFloater* b);
    double approx_distance(StateFloater* a, StateFloater* b);

    void setRandomStateConstraints(StateFloater _minimums, StateFloater _maximums);
    StateFloater getRandomState();

protected:
    StateFloater minimums, maximums;
    StateFloater scale, shift;

    const float EDGE_WALK_SCALE = 1.0f;

    MapFloater* map = nullptr;

};

#endif