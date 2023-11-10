#include "StateFloaterMath.h"
#include <cmath>
#include <motion/Motion1DPositionVelocitySingleTimed.h>

///////////////////////////////////////////////  SETUP  //////////////////////////////////////////////////

StateFloaterMath::StateFloaterMath(float _max_velocity, float _max_accel) {
    max_velocity = _max_velocity;
    max_accel = _max_accel;
}

void StateFloaterMath::setMap(MapFloater *_map) {
    map = _map;
    StateFloater _minimums, _maximums;
    map->getBounds(&_minimums, &_maximums);
    _minimums.vy = -max_velocity;
    _maximums.vy = max_velocity;
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

    Motion1DPositionVelocitySingleTimed motion;

    motion.configure_velocity_limits(-max_velocity, max_velocity);
    motion.configure_acceleration_limits(-max_accel, max_accel);
    motion.configure_dt(0.1);

    motion.jump_to(pointA->y, pointA->vy);
    motion.set_target(pointB->y, pointB->vy);
    motion.set_arrival_time(pointB->t - pointA->t);

    motion.run_next_timestep();

    float t[8] = {0}, a[8] = {0}, j[8] = {0};
    motion.get_last_solution(t, a, j);

    // integrate the jerk to calculate total energy spent

    float ji = 0;
    for (int i=1; i<8; i++) {
        float dt = t[i] - t[i-1];
        float jsum = j[i-1] * dt;
        ji += jsum;
    }

    // if no solution can be found we'll get a sum of zero
    if (ji == 0) ji = INFINITY;

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
    scale.t = (maximums.t - minimums.t - 1) / RAND_MAX;
    scale.y = (maximums.y - minimums.y - 1) / RAND_MAX;
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
