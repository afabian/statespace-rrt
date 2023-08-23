#include "StateFloaterMath.h"
#include <cmath>

// "Floater" is a sample system, with a hypothetical vehicle that moves forward
// at a constant rate (so we could say it's position x equals time t), but
// is free to move vertically.  In the vertical (y) axis, it has gravity and inertia,
// and can command update thrust between 0% and 100% of some maximum value.
//
// Maps are developed with obstacles in x-y space, so that the vehicle must
// time its thrust commands to avoid the obstacle.

double StateFloaterMath::distance(StateFloater* a, StateFloater* b) {
    double dt = a->t - b->t;
    double dy = a->y - b->y;
    double dvy = a->vy - b->vy;
    double dist = sqrt(dt*dt + dy*dy + dvy*dvy);
    return dist;
}

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
