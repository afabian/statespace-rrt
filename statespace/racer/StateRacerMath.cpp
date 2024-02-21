#include "StateRacerMath.h"
#include <cmath>
#include <iostream>

///////////////////////////////////////////////  SETUP  //////////////////////////////////////////////////

StateRacerMath::StateRacerMath(float _max_velocity, float _max_lateral_accel) {
    max_velocity = _max_velocity;
    max_lateral_accel = _max_lateral_accel;
}

void StateRacerMath::setMap(MapRacer *_map) {
    map = _map;
    StateRacer _minimums, _maximums;
    map->getBounds(&_minimums, &_maximums);
    setRandomStateConstraints(_minimums, _maximums);
    generateStateTransitionLUT();
}

////////////////////////////////////////// MODEL SIMULATION //////////////////////////////////////////////

void StateRacerMath::generateStateTransitionLUT() {
    // do forward simulations of the model, iterating over starting states and possible internal control inputs
    // to generate a map of output states vs. input states.
    // the main RRT loop will use this map to figure out if two states are connectable, and what the cost is.

    for (float vi = 0; vi < max_velocity; vi += max_velocity / velocity_steps) {

    }
}

////////////////////////////////////////  OBSTACLE DETECTION  ////////////////////////////////////////////

bool StateRacerMath::pointInObstacle(StateRacer *point) {
    return map->getPixelIsObstacle(point->x, point->y);
}

bool StateRacerMath::edgeInObstacle(StateRacer *source, StateRacer *dest) {
    StateRacer diff(dest->x - source->x, dest->y - source->y);
    float step = EDGE_WALK_SCALE / hypotf(diff.x, diff.y);
    for (float progress = 0; progress < 1; progress += step) {
        StateRacer point(source->x + diff.x * progress, source->y + diff.y * progress);
        if (pointInObstacle(&point)) {
            return true;
        }
    }
    return false;
}

/////////////////////////////////////////  COST CALCULATIONS  ////////////////////////////////////////////

float StateRacerMath::edgeCost(StateRacer *source, StateRacer *dest) {
}

void StateRacerMath::edgePath(StateRacer *source, StateRacer *dest, float t[], float p[], float a[], float pointCount) {
}

///////////////////////////////////////  DISTANCE CALCULATIONS  //////////////////////////////////////////

double StateRacerMath::distance(StateRacer *source, StateRacer *dest) {
    double dx = source->x - dest->x;
    double dy = source->y - dest->y;
    double dist = hypot(dx, dy);
    return dist;
}

double StateRacerMath::approx_distance(StateRacer *source, StateRacer *dest) {
    return fabs(dest->x - source->x) + fabs(dest->y - source->y);
}

////////////////////////////////////////// SAMPLE GENERATION /////////////////////////////////////////////

void StateRacerMath::setRandomStateConstraints(StateRacer _minimums, StateRacer _maximums) {
    minimums = _minimums;
    maximums = _maximums;
    // scale and shift are optimized to make getRandomState() fast
    scale.x = (maximums.x - minimums.x - 1) / RAND_MAX;
    scale.y = (maximums.y - minimums.y - 1) / RAND_MAX;
    scale.v = max_velocity;
    scale.h = 360;
    shift.x = minimums.x;
    shift.y = minimums.y;
    shift.v = 0;
    shift.h = 0;
}

StateRacer StateRacerMath::getRandomState() {
    StateRacer output;
    output.x = (double)rand() * scale.x + shift.x;
    output.y = (double)rand() * scale.y + shift.y;
    output.v = (double)rand() * scale.v + shift.v;
    output.h = (double)rand() * scale.h + shift.h;
    return output;
}
