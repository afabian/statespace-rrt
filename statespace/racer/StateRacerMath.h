#ifndef STATERACERMATH_H
#define STATERACERMATH_H

#include "StateRacer.h"
#include "MapRacer.h"

// Racer is a top-down racing game solver, where the car can accelerate, brake, coast, go straight or left or right.

class MapRacer;

class StateRacerMath {

public:
    StateRacerMath(float _max_velocity, float _max_lateral_accel);

    void setMap(MapRacer* _map);

    bool pointInObstacle(StateRacer* point);
    bool edgeInObstacle(StateRacer* source, StateRacer* dest);

    float edgeCost(StateRacer* source, StateRacer* dest);

    void edgePath(StateRacer *source, StateRacer *dest, float t[], float p[], float a[], float pointCount);

    double distance(StateRacer* source, StateRacer* dest);
    double approx_distance(StateRacer* source, StateRacer* dest);

    void setRandomStateConstraints(StateRacer _minimums, StateRacer _maximums);
    StateRacer getRandomState();

protected:
    void generateStateTransitionLUT();

    StateRacer minimums, maximums;
    StateRacer scale, shift;

    const float EDGE_WALK_SCALE = 1.0f;

    MapRacer* map = nullptr;

    float max_velocity = 1;
    float max_lateral_accel = 1;

};

#endif