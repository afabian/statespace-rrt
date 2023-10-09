#include "State3DMath.h"
#include <cmath>

///////////////////////////////////////////////  SETUP  //////////////////////////////////////////////////

State3DMath::State3DMath() {
    cost_scale = 1;
}

State3DMath::State3DMath(float scale) {
    cost_scale = scale;
}

void State3DMath::setMap(Map3D *_map) {
    map = _map;
    State3D _minimums, _maximums;
    map->getBounds(&_minimums, &_maximums);
    setRandomStateConstraints(_minimums, _maximums);
}

void State3DMath::setCostScale(float _scale) {
    cost_scale = _scale;
}

////////////////////////////////////////  OBSTACLE DETECTION  ////////////////////////////////////////////

bool State3DMath::pointInObstacle(State3D *point) {
    for (int i=0; i<map->object_count; i++) {
        bool x_inside = point->x > map->objects[i].bound_lower.x && point->x < map->objects[i].bound_upper.x;
        bool y_inside = point->y > map->objects[i].bound_lower.y && point->y < map->objects[i].bound_upper.y;
        bool z_inside = point->z > map->objects[i].bound_lower.z && point->z < map->objects[i].bound_upper.z;
        if (x_inside && y_inside && z_inside) {
            return true;
        }
    }
    return false;
}

bool State3DMath::edgeInObstacle(State3D *pointA, State3D *pointB) {
    State3D diff(pointB->x - pointA->x, pointB->y - pointA->y, pointB->z - pointA->z);
    float step = EDGE_WALK_SCALE / sqrtf(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
    for (float progress = 0; progress < 1; progress += step) {
        State3D point(pointA->x + diff.x * progress, pointA->y + diff.y * progress, pointA->z + diff.z * progress);
        if (pointInObstacle(&point)) {
            return true;
        }
    }
    return false;
}

/////////////////////////////////////////  COST CALCULATIONS  ////////////////////////////////////////////

float State3DMath::pointCost(State3D *point) {
    if (point->x < map->border.bound_lower.x || point->x > map->border.bound_upper.x) return INFINITY;
    if (point->y < map->border.bound_lower.y || point->x > map->border.bound_upper.y) return INFINITY;
    if (point->z < map->border.bound_lower.z || point->x > map->border.bound_upper.z) return INFINITY;
    return pointInObstacle(point) ? INFINITY : 1;
}

float State3DMath::edgeCost(State3D *pointA, State3D *pointB) {
    float sum = 0;
    State3D diff(pointB->x - pointA->x, pointB->y - pointA->y, pointB->z - pointA->z);
    int iterations = 0;
    float length = sqrtf(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
    for (float progress = 0; progress < 1; progress += EDGE_WALK_SCALE / length) {
        State3D point(pointA->x + diff.x * progress, pointA->y + diff.y * progress, pointA->z + diff.z * progress);
        sum += pointCost(&point);
        iterations++;
    }
    return iterations == 0 ? 0 : sum / iterations * length;
}

///////////////////////////////////////  DISTANCE CALCULATIONS  //////////////////////////////////////////

double State3DMath::distance(State3D* a, State3D* b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double dz = a->z - b->z;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    return dist;
}

double State3DMath::approx_distance(State3D* a, State3D* b) {
    return fabs(a->x - b->x) + fabs(a->y - b->y) + fabs(a->z - b->z);
}

////////////////////////////////////////// SAMPLE GENERATION /////////////////////////////////////////////

void State3DMath::setRandomStateConstraints(State3D _minimums, State3D _maximums) {
    minimums = _minimums;
    maximums = _maximums;
    // scale and shift are optimized to make getRandomState() fast
    scale.x = (maximums.x - minimums.x) / RAND_MAX;
    scale.y = (maximums.y - minimums.y) / RAND_MAX;
    scale.z = (maximums.z - minimums.z) / RAND_MAX;
    shift.x = -minimums.x;
    shift.y = -minimums.y;
    shift.z = -minimums.z;
}

State3D State3DMath::getRandomState() {
    State3D output;
    output.x = (double)rand() * scale.x + shift.x;
    output.y = (double)rand() * scale.y + shift.y;
    output.z = (double)rand() * scale.z + shift.z;
    return output;
}
