#include "StateFloaterMath.h"
#include <cmath>
#include <motion/Motion1DPositionVelocityAccelSingleTimed.h>

///////////////////////////////////////////////  SETUP  //////////////////////////////////////////////////

StateFloaterMath::StateFloaterMath() {
}

void StateFloaterMath::setMap(MapFloater *_map) {
    map = _map;
    StateFloater _minimums, _maximums;
    map->getBounds(&_minimums, &_maximums);
    setRandomStateConstraints(_minimums, _maximums);
}

////////////////////////////////////////  OBSTACLE DETECTION  ////////////////////////////////////////////

bool StateFloaterMath::pointInObstacle(StateFloater *point) {
    return map->getGrayscalePixel(point->t, point->y) < 0.01;
}

bool StateFloaterMath::edgeInObstacle(StateFloater *pointA, StateFloater *pointB) {
    StateFloater diff(pointB->t - pointA->t, pointB->y - pointA->y, 0);
    float step = EDGE_WALK_SCALE / hypotf(diff.t, diff.y);
    for (float progress = 0; progress < 1; progress += step) {
        StateFloater point(pointA->t + diff.t * progress, pointA->y + diff.y * progress, 0);
        if (pointInObstacle(&point)) {
            return true;
        }
    }
    return false;
}

/////////////////////////////////////////  COST CALCULATIONS  ////////////////////////////////////////////

float StateFloaterMath::edgeCost(StateFloater *pointA, StateFloater *pointB) {

    // check for impossible scenarios and return an infinite cost

    if (pointB->t < pointA->t) return INFINITY;
    if (pointB->t == pointA->t && (pointB->y != pointA->y || pointB->vy != pointA->vy)) return INFINITY;

    // the cost is the amount of energy/thrust it takes to move from the initial state to the final state
    // not all moves are possible, and impossible moves return infinity

    Motion1DPositionVelocityAccelSingleTimed motion;

    motion.configure_acceleration_limits(9.8, 5);
    motion.configure_dt(1);

    motion.jump_to(pointA->y, pointA->vy);
    motion.set_target(pointB->y, pointB->vy);
    motion.set_arrival_time(pointB->t - pointA->t);

    motion.run_next_timestep();

    float t[8] = {0}, a[8] = {0}, j[8] = {0};
    motion.get_last_solution(t, a, j);

    // if the solution is invalid, return an infinite cost



    // integrate the jerk to calculate total energy spent

    float ji = 0;
    for (int i=1; i<8; i++) {
        float dt = t[i] - t[i-1];
        float jsum = j[i-1] * dt;
        ji += jsum;
    }

    return ji;

}

///////////////////////////////////////  DISTANCE CALCULATIONS  //////////////////////////////////////////

double StateFloaterMath::distance(StateFloater *a, StateFloater *b) {
    double dt = a->t - b->t;
    double dy = a->y - b->y;
    double dvy = a->vy - b->vy;
    double dist = sqrt(dt*dt + dy*dy + dvy*dvy);
    return dist;
}

double StateFloaterMath::approx_distance(StateFloater *a, StateFloater *b) {
    return 0;
}

////////////////////////////////////////// SAMPLE GENERATION /////////////////////////////////////////////

void StateFloaterMath::setRandomStateConstraints(StateFloater _minimums, StateFloater _maximums) {
    minimums = _minimums;
    maximums = _maximums;
    // scale and shift are optimized to make getRandomState() fast
    scale.t = (maximums.t - minimums.t) / RAND_MAX;
    scale.y = (maximums.y - minimums.y) / RAND_MAX;
    scale.vy = (maximums.vy - minimums.vy) / RAND_MAX;
    shift.t = -minimums.t;
    shift.y = -minimums.y;
    shift.vy = -minimums.vy;
}

StateFloater StateFloaterMath::getRandomState() {
    StateFloater output;
    output.t = (double)rand() * scale.t + shift.t;
    output.y = (double)rand() * scale.y + shift.y;
    output.vy = (double)rand() * scale.vy + shift.vy;
    return output;
}
