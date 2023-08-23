#include "State2DMath.h"
#include <cmath>

double State2DMath::distance(State2D* a, State2D* b) {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double dist = hypot(dx, dy);
    return dist;
}

double State2DMath::approx_distance(State2D* a, State2D* b) {
    return fabs(a->x - b->x) + fabs(a->y - b->y);
}

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
