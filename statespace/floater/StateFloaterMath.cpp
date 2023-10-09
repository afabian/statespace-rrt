#include "StateFloaterMath.h"
#include <cmath>

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

float StateFloaterMath::pointCost(StateFloater *point) {
    return 0;
}

float StateFloaterMath::edgeCost(StateFloater *pointA, StateFloater *pointB) {
    return 0;
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
