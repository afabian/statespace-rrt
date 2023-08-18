#include "State3DMath.h"
#include <cmath>

double State3DMath::distance(State3D* a, State3D* b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double dz = a->z - b->z;
    double dist = sqrt(dx*dx + dy*dy + dz*dz);
    return dist;
}

bool State3DMath::connectable(State3D* a, State3D* b)  {
    return true;
}

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
