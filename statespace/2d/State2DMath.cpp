#include "State2DMath.h"
#include <cmath>

///////////////////////////////////////////////  SETUP  //////////////////////////////////////////////////

State2DMath::State2DMath() {
    cost_scale = 1;
}

State2DMath::State2DMath(float scale) {
    cost_scale = scale;
}

void State2DMath::setMap(Map2D *_map) {
    map = _map;
    State2D _minimums, _maximums;
    map->getBounds(&_minimums, &_maximums);
    setRandomStateConstraints(_minimums, _maximums);
}

void State2DMath::setCostScale(float _scale) {
    cost_scale = _scale;
}

////////////////////////////////////////  OBSTACLE DETECTION  ////////////////////////////////////////////

bool State2DMath::pointInObstacle(State2D *point) {
    return map->getGrayscalePixel(point->x, point->y) < 0.01;
}

bool State2DMath::edgeInObstacle(State2D *pointA, State2D *pointB) {
    State2D diff(pointB->x - pointA->x, pointB->y - pointA->y);
    float step = EDGE_WALK_SCALE / hypotf(diff.x, diff.y);
    for (float progress = 0; progress < 1; progress += step) {
        State2D point(pointA->x + diff.x * progress, pointA->y + diff.y * progress);
        if (pointInObstacle(&point)) {
            return true;
        }
    }
    return false;
}

/////////////////////////////////////////  COST CALCULATIONS  ////////////////////////////////////////////

float State2DMath::pointCost(State2D *point) {
    int x = point->x;
    int y = point->y;
    if (x < 0 || x >= maximums.x || y < 0 || y >= maximums.y) {
        return INFINITY;
    }
    else {
        return 1.0f + (cost_scale * (1.0f - map->getGrayscalePixel(x, y)));
    }
}

float State2DMath::edgeCost(State2D *pointA, State2D *pointB) {
    float sum = 0;
    State2D diff(pointB->x - pointA->x, pointB->y - pointA->y);
    int iterations = 0;
    float length = hypotf(diff.x, diff.y);
    for (float progress = 0; progress < 1; progress += EDGE_WALK_SCALE / length) {
        State2D point(pointA->x + diff.x * progress, pointA->y + diff.y * progress);
        sum += pointCost(&point);
        iterations++;
    }
    return iterations == 0 ? 0 : sum / iterations * length;
}

///////////////////////////////////////  DISTANCE CALCULATIONS  //////////////////////////////////////////

double State2DMath::distance(State2D* a, State2D* b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double dist = hypot(dx, dy);
    return dist;
}

double State2DMath::approx_distance(State2D* a, State2D* b) {
    return fabs(a->x - b->x) + fabs(a->y - b->y);
}

////////////////////////////////////////// SAMPLE GENERATION /////////////////////////////////////////////

void State2DMath::setRandomStateConstraints(State2D _minimums, State2D _maximums) {
    minimums = _minimums;
    maximums = _maximums;
    // scale and shift are optimized to make getRandomState() fast
    scale.x = (maximums.x - minimums.x) / RAND_MAX;
    scale.y = (maximums.y - minimums.y) / RAND_MAX;
    shift.x = -minimums.x;
    shift.y = -minimums.y;
}

State2D State2DMath::getRandomState() {
    State2D output;
    output.x = (double)rand() * scale.x + shift.x;
    output.y = (double)rand() * scale.y + shift.y;
    return output;
}
