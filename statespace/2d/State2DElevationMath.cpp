#include "State2DElevationMath.h"
#include <cmath>

///////////////////////////////////////////////  SETUP  //////////////////////////////////////////////////

State2DElevationMath::State2DElevationMath() {
    cost_scale = 1;
}

State2DElevationMath::State2DElevationMath(float scale) {
    cost_scale = scale;
}

void State2DElevationMath::setMap(Map2D *_map) {
    map = _map;
    State2D _minimums, _maximums;
    map->getBounds(&_minimums, &_maximums);
    setRandomStateConstraints(_minimums, _maximums);
}

void State2DElevationMath::setCostScale(float scale) {
    cost_scale = scale;
}

////////////////////////////////////////  OBSTACLE DETECTION  ////////////////////////////////////////////

bool State2DElevationMath::pointInObstacle(State2D *point) {
    return map->getGrayscalePixel(point->x, point->y) < 0.01;
}

bool State2DElevationMath::edgeInObstacle(State2D *pointA, State2D *pointB) {
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

float State2DElevationMath::pointCost(State2D *point, float heading) {
    int x = point->x;
    int y = point->y;
    if (x < 1 || x >= maximums.x || y < 1 || y >= maximums.y) {
        return INFINITY;
    }
    else {
        float dx = map->getGrayscalePixel(x, y) - map->getGrayscalePixel(x-1, y);
        float dy = map->getGrayscalePixel(x, y) - map->getGrayscalePixel(x, y-1);
        float costx = fabsf(dx * cosf(heading));
        float costy = fabsf(dy * sinf(heading));
        return 1.0f + (cost_scale * (costx + costy));
    }
}

float State2DElevationMath::edgeCost(State2D *pointA, State2D *pointB, State2D *pointB_updated) {
    float sum = 0;
    State2D diff(pointB->x - pointA->x, pointB->y - pointA->y);
    int iterations = 0;
    float length = hypotf(diff.x, diff.y);
    float heading = atan2f(pointB->y-pointA->y, pointB->x-pointA->x);
    for (float progress = 0; progress < 1; progress += EDGE_WALK_SCALE / length) {
        State2D point(pointA->x + diff.x * progress, pointA->y + diff.y * progress);
        sum += pointCost(&point, heading);
        iterations++;
    }
    if (pointB_updated != nullptr) {
        *pointB_updated = *pointB;
    }
    return iterations == 0 ? 0 : sum / iterations * length;
}

///////////////////////////////////////  DISTANCE CALCULATIONS  //////////////////////////////////////////

double State2DElevationMath::distance(State2D* a, State2D* b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double dist = hypot(dx, dy);
    return dist;
}

double State2DElevationMath::approx_distance(State2D* a, State2D* b) {
    return fabs(a->x - b->x) + fabs(a->y - b->y);
}

////////////////////////////////////////// SAMPLE GENERATION /////////////////////////////////////////////

void State2DElevationMath::setRandomStateConstraints(State2D _minimums, State2D _maximums) {
    minimums = _minimums;
    maximums = _maximums;
    // scale and shift are optimized to make getRandomState() fast
    scale.x = (maximums.x - minimums.x - 1) / RAND_MAX;
    scale.y = (maximums.y - minimums.y - 1) / RAND_MAX;
    shift.x = minimums.x;
    shift.y = minimums.y;
}

State2D State2DElevationMath::getRandomState() {
    State2D output;
    output.x = (double)rand() * scale.x + shift.x;
    output.y = (double)rand() * scale.y + shift.y;
    return output;
}
