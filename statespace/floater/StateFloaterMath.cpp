#include "StateFloaterMath.h"
#include <cmath>
#include <iostream>

///////////////////////////////////////////////  SETUP  //////////////////////////////////////////////////

StateFloaterMath::StateFloaterMath(float _max_velocity, float _max_accel) {
    max_velocity = _max_velocity;
    max_accel = _max_accel;
}

void StateFloaterMath::setMap(MapFloater *_map) {
    map = _map;
    map->setStateFloaterMath(this);
    StateFloater _minimums, _maximums;
    map->getBounds(&_minimums, &_maximums);
    _minimums.vy = -max_velocity;
    _maximums.vy = max_velocity;
    setRandomStateConstraints(_minimums, _maximums);
}

////////////////////////////////////////  OBSTACLE DETECTION  ////////////////////////////////////////////

bool StateFloaterMath::pointInObstacle(StateFloater *point) {
    float grayscale = map->getGrayscalePixel(point->t, point->y);
    return grayscale < 0.01;
}

bool StateFloaterMath::edgeInObstacle(StateFloater *source, StateFloater *dest) {
    bool in_obstacle = false;
    int points = dest->t - source->t;
    float *y = (float *) malloc(sizeof(float) * points);
    float *t = (float *) malloc(sizeof(float) * points);
    edgePath(source, dest, t, y, points);
    for (int i = 0; i < points; i++) {
        StateFloater point(t[i], y[i], 0);
        if (pointInObstacle(&point)) {
            in_obstacle = true;
            break;
        };
    }
    free(y);
    free(t);
    return in_obstacle;
}

/////////////////////////////////////////  COST CALCULATIONS  ////////////////////////////////////////////

float StateFloaterMath::edgeCost(StateFloater *source, StateFloater *dest) {

    // check for impossible scenarios and return an infinite cost

    if (dest->t < source->t) return INFINITY;
    if (dest->t == source->t && (dest->y != source->y || dest->vy != source->vy)) return INFINITY;

    // the cost is the amount of energy/thrust it takes to move from the initial state to the final state
    // not all moves are possible, and impossible moves return infinity

    Motion1DPositionVelocityAccelSingleTimed motion;
    configureMotionPlanner(&motion, source, dest);
    motion.run_next_timestep();

    float t[8] = {0}, a[8] = {0}, j[8] = {0};
    motion.get_last_solution(t, a, j);

    // integrate the acceleration over time to calculate total energy spent

    float ai = 0;
    for (int i=1; i<8; i++) {
        float dt = t[i] - t[i-1];
        float asum = fabsf(a[i-1]) * dt;
        ai += asum;
    }

    // if no solution can be found we'll get a sum of zero
    if (ai == 0) ai = INFINITY;
    if (motion.get_error_code().value != 0) ai = INFINITY;

    // see if the timespan covered by the path matches the timespan between our source and dest
    float solution_time = t[3];
    float problem_time = dest->t - source->t;
    float time_ratio = problem_time == 0 ? 0 : solution_time / problem_time;
    if (time_ratio > 1.01f || time_ratio < 0.99f) ai = INFINITY;

    return ai;
}

void StateFloaterMath::edgePath(StateFloater *source, StateFloater *dest, float t[], float y[], float pointCount) {
    Motion1DPositionVelocityAccelSingleTimed motion;
    configureMotionPlanner(&motion, source, dest);
    motion.reset_state();
    // this should include a point at the source point and at the destination point, as well as intermediate points
    float dt = (dest->t - source->t) / (pointCount-1);
    motion.configure_dt(dt);
    for (int i=0; i<pointCount; i++) {
        t[i] = source->t + i * dt;
        y[i] = motion.get_position();
        motion.run_next_timestep();
    }
}

void StateFloaterMath::configureMotionPlanner(Motion1DPositionVelocityAccelSingleTimed *motion, StateFloater *source, StateFloater *dest) {
    motion->configure_velocity_limits(-max_velocity, max_velocity);
    motion->configure_acceleration_limits(-max_accel, max_accel);
    motion->configure_dt(0.1);

    motion->jump_to(source->y, source->vy);
    motion->set_target(dest->y, dest->vy);
    motion->set_arrival_time(dest->t - source->t);
}

///////////////////////////////////////  DISTANCE CALCULATIONS  //////////////////////////////////////////

double StateFloaterMath::distance(StateFloater *source, StateFloater *dest) {
    double dt = dest->t - source->t;
    double dy = dest->y - source->y;
    double dvy = dest->vy - source->vy;
    if (dvy == INFINITY || dvy == -INFINITY) dvy = 0;
    if (dt < 0) return INFINITY;
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
    shift.t = minimums.t;
    shift.y = minimums.y;
    shift.vy = minimums.vy;
}

StateFloater StateFloaterMath::getRandomState() {
    StateFloater output;
    output.t = (double)rand() * scale.t + shift.t;
    output.y = (double)rand() * scale.y + shift.y;
    output.vy = (double)rand() * scale.vy + shift.vy;
    return output;
}
