#ifndef STATEFLOATERMATH_H
#define STATEFLOATERMATH_H

#include "StateFloater.h"
#include "MapFloater.h"
#include <motion/Motion1DPositionVelocityAccelSingleTimed.h>

// "Floater" is a sample system, with a hypothetical vehicle that moves forward
// at a constant rate (so we could say it's position x equals time t), but
// is free to move vertically.  In the vertical (y) axis, it has gravity and inertia,
// and can command update thrust between 0% and 100% of some maximum value.
//
// Maps are developed with obstacles in x-y space, so that the vehicle must
// time its thrust commands to avoid the obstacle.

class MapFloater;

class StateFloaterMath {
  
public:
    StateFloaterMath(float _max_velocity, float _max_accel);

    void setMap(MapFloater* _map);

    bool pointInObstacle(StateFloater* point);
    bool edgeInObstacle(StateFloater* source, StateFloater* dest);

    float edgeCost(StateFloater* source, StateFloater* dest, StateFloater* dest_updated=nullptr);

    void edgePath(StateFloater *source, StateFloater *dest, float t[], float p[], float a[], float pointCount);

    double distance(StateFloater* source, StateFloater* dest);
    double approx_distance(StateFloater* a, StateFloater* b);

    void setRandomStateConstraints(StateFloater _minimums, StateFloater _maximums);
    StateFloater getRandomState();

protected:
    void configureMotionPlanner(Motion1DPositionVelocityAccelSingleTimed* motion, StateFloater* source, StateFloater* dest);

    StateFloater minimums, maximums;
    StateFloater scale, shift;

    const float EDGE_WALK_SCALE = 1.0f;

    MapFloater* map = nullptr;

    float max_velocity = 1;
    float max_accel = 1;

};

#endif